/**
 * CurrentReadback.js
 * 求解完成后的电流/物理量回传
 * 将 MNA results 向量中的数据写回各器件的 physCurrent / ch1Current 等属性
 */

export const CurrentReadback = {

    /**
     * 线性电阻电流（由节点电压差计算）
     */
    readResistors(nodeVoltages, portToCluster, resistorDevs) {
        resistorDevs.forEach(dev => {
            if (dev.currentResistance < 0.1) return;
            const vL = nodeVoltages.get(portToCluster.get(`${dev.id}_wire_l`)) || 0;
            const vR = nodeVoltages.get(portToCluster.get(`${dev.id}_wire_r`)) || 0;
            dev.physCurrent = (vL - vR) / dev.currentResistance;
        });
    },

    /**
     * 压力传感器各支路电流
     */
    readPressureSensors(nodeVoltages, portToCluster, pressDevs) {
        pressDevs.forEach(dev => {
            const c1l = portToCluster.get(`${dev.id}_wire_r1l`);
            const c1r = portToCluster.get(`${dev.id}_wire_r1r`);
            const c2l = portToCluster.get(`${dev.id}_wire_r2l`);
            const c2r = portToCluster.get(`${dev.id}_wire_r2r`);

            dev.r1Current = ((nodeVoltages.get(c1l) || 0) - (nodeVoltages.get(c1r) || 0)) / Math.max(0.001, dev.r1);
            dev.r2Current = ((nodeVoltages.get(c2l) || 0) - (nodeVoltages.get(c2r) || 0)) / Math.max(0.001, dev.r2);
        });
    },

    /**
     * 变送器压差缓存（供下一帧使用）
     */
    readTransmitters(getVoltageAtPort, transmitterDevs) {
        transmitterDevs.forEach(dev => {
            const pV = getVoltageAtPort(`${dev.id}_wire_p`);
            const nV = getVoltageAtPort(`${dev.id}_wire_n`);
            dev._lastVDiff = pV - nV;
        });
    },

    /**
     * PID 电流回传（兼容电压源模式和恒流源模式）
     */
    readPIDs(results, nodeVoltages, portToCluster, clusters, getEquivR, pidDevs) {
        const nNodes = results.length; // 仅用于文档说明，实际索引由 vSourceIdx 存储

        pidDevs.forEach(pid => {
            if (!pid.powerOn) return;

            // CH1
            if (pid._ch1CurrentInfo) {
                const info = pid._ch1CurrentInfo;
                if (info.mode === 'voltage') {
                    pid.ch1Current = Math.abs(results[info.index]) * 1000;
                } else {
                    pid.ch1Current = info.valueA * 1000;
                }
                pid._ch1CurrentInfo = null;
            } else if (pid.ch1VSourceIdx !== undefined) {
                // PWM 模式：直接读电压源电流
                pid.ch1Current = results[pid.ch1VSourceIdx];
            }

            // CH2
            if (pid._ch2CurrentInfo) {
                const info = pid._ch2CurrentInfo;
                if (info.mode === 'voltage') {
                    pid.ch2Current = Math.abs(results[info.index]) * 1000;
                } else {
                    pid.ch2Current = info.valueA * 1000;
                }
                pid._ch2CurrentInfo = null;
            } else if (pid.ch2VSourceIdx !== undefined) {
                pid.ch2Current = results[pid.ch2VSourceIdx];
            }
        });
    },

    /**
     * 热电偶电流（流过电压源的电流）
     */
    readThermocouples(results, tcDevs) {
        tcDevs.forEach(tc => {
            if (tc.vSourceIdx !== undefined) tc.physCurrent = results[tc.vSourceIdx];
        });
    },

    /**
     * 运放输出电流
     */
    readOpAmps(results, opAmps) {
        opAmps.forEach(op => {
            if (op.currentIdx !== undefined) op.outCurrent = results[op.currentIdx];
        });
    },

    /**
     * 二极管电流（由节点电压重新计算，与注入逻辑保持一致）
     */
    readDiodes(nodeVoltages, portToCluster, diodeDevs) {
        diodeDevs.forEach(dev => {
            const cA = portToCluster.get(`${dev.id}_wire_l`);
            const cC = portToCluster.get(`${dev.id}_wire_r`);
            const vA = nodeVoltages.get(cA) || 0;
            const vC = nodeVoltages.get(cC) || 0;
            const vDiff = vA - vC;
            const vForward = dev.vForward || 0.68;
            const rOn = dev.rOn || 0.5;

            dev.physCurrent = (vDiff > vForward) ? (1 / rOn) * (vDiff - vForward) : 0;
        });
    },

    /**
     * BJT 三极管各极电流
     */
    readBJTs(nodeVoltages, portToCluster, bjtDevs) {
        bjtDevs.forEach(dev => {
            const cB = portToCluster.get(`${dev.id}_wire_b`);
            const cC = portToCluster.get(`${dev.id}_wire_c`);
            const cE = portToCluster.get(`${dev.id}_wire_e`);
            const vB = nodeVoltages.get(cB) || 0;
            const vC = nodeVoltages.get(cC) || 0;
            const vE = nodeVoltages.get(cE) || 0;

            dev.physCurrents = { b: 0, c: 0, e: 0 };
            const model = dev.getCompanionModel(vB, vC, vE);
            const { gBE, iBE, beta, gCE_sat, pol, V_SAT } = model.internal;

            if (cB !== undefined && cE !== undefined && (cC === undefined || cC === cB)) {
                const vDiff = (vB - vE) * pol;
                const Ib = (vDiff > 0.7) ? 2 * (vDiff - 0.7) : 0;
                dev.physCurrents.b = Ib * pol;
                dev.physCurrents.e = -dev.physCurrents.b;
            } else if (cB !== undefined && cC !== undefined && (cE === undefined || cE === cB)) {
                const vDiff = (vB - vC) * pol;
                const Ib = (vDiff > 0.7) ? 2 * (vDiff - 0.7) : 0;
                dev.physCurrents.b = Ib * pol;
                dev.physCurrents.c = -dev.physCurrents.b;
            } else {
                const vbeLocal = (vB - vE) * pol;
                const vceLocal = (vC - vE) * pol;
                const Ib = pol * (gBE * vbeLocal + iBE);
                const Ic = (beta * Ib) + pol * (gCE_sat * (vceLocal - V_SAT));
                dev.physCurrents.b = Ib;
                dev.physCurrents.c = Ic;
                dev.physCurrents.e = -(Ib + Ic);
            }
        });
    },

    /**
     * NJFET 电流
     */
    readJFETs(nodeVoltages, portToCluster, jfetDevs) {
        jfetDevs.forEach(dev => {
            const cD = portToCluster.get(`${dev.id}_wire_d`);
            const cS = portToCluster.get(`${dev.id}_wire_s`);
            const vD = nodeVoltages.get(cD) || 0;
            const vS = nodeVoltages.get(cS) || 0;
            const res = dev.getDSResistance(vD - vS);
            dev.physCurrent = (vD - vS) / res;
        });
    },

    /**
     * 电容 / 电感：更新历史状态
     */
    readAndUpdateReactives(nodeVoltages, portToCluster, devs, deltaTime, isInductor = false) {
        devs.forEach(dev => {
            const cL = portToCluster.get(`${dev.id}_wire_l`);
            const cR = portToCluster.get(`${dev.id}_wire_r`);
            const vL = nodeVoltages.get(cL) || 0;
            const vR = nodeVoltages.get(cR) || 0;

            dev.calculatePhysicalCurrent(vL, vR, deltaTime);
            if (isInductor) dev.updateState();
            else dev.updateState(vL, vR);
        });
    },

    /**
     * 示波器电流
     */
    readOscilloscopes(results, oscDevs) {
        oscDevs.forEach(dev => {
            if (dev.currentIdx !== undefined) dev.physCurrent = results[dev.currentIdx];
        });
    },

    /**
     * LVDT / 压力变送器输出电流
     */
    readLVDTs(results, lvdtDevs) {
        lvdtDevs.forEach(dev => {
            if (dev.currentIdx !== undefined) dev.physCurrent = results[dev.currentIdx];
        });
    },

    /**
     * 信号发生器输出电流（由节点电压差计算）
     */
    readSignalGenerators(nodeVoltages, portToCluster, sgDevs) {
        sgDevs.forEach(sg => {
            [
                { key: 'ch1', p: 'ch1p', n: 'ch1n', idx: 0 },
                { key: 'ch2', p: 'ch2p', n: 'ch2n', idx: 1 }
            ].forEach(chCfg => {
                const ch = sg.channels[chCfg.idx];
                const portP = portToCluster.get(`${sg.id}_wire_${chCfg.p}`);
                const portN = portToCluster.get(`${sg.id}_wire_${chCfg.n}`);

                if (ch.enabled && portP !== undefined && portN !== undefined) {
                    const vP = nodeVoltages.get(portP) || 0;
                    const vN = nodeVoltages.get(portN) || 0;
                    const Vs = sg.voltOutputs[chCfg.key];
                    const Rs = 50;
                    const current = (Vs - (vP - vN)) / Rs;
                    if (chCfg.idx === 0) sg.ch1Current = current;
                    else sg.ch2Current = current;
                } else {
                    if (chCfg.idx === 0) sg.ch1Current = 0;
                    else sg.ch2Current = 0;
                }
            });
        });
    },
};
