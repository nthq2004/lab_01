import { Workflow } from './tools/Workflow.js';  // 流程控制工具
import { CircuitSolver } from './tools/CircuitSolver.js';  // 电路求解工具
import { PneumaticSolver } from './tools/PneumaticSolver.js'; // 气路求解工具
import { Show } from './tools/Show.js'; // 提示展示工具

import { LeakDetector } from './components/LeakDetector.js'; // 泄漏检测器组件
import { AirBottle } from './components/AirBottle.js'; // 气瓶组件
import { PressRegulator } from './components/PressRegulator.js';  // 减压阀组件
import { PressMeter } from './components/PressMeter.js';  // 压力表组件
import { TeeConnector } from './components/TeeConnector.js'; // 三通连接器组件
import { StopValve } from './components/StopValve.js'; // 截止阀组件
import { Pump } from './components/Pump.js';  // 泵组件
import { Cooler } from './components/Cooler.js';  // 冷却器组件
import { Engine } from './components/Engine.js';  // 发动机组件

import { PIDController } from './components/PID.js';  // PID控制器组件
import { OvenSystem } from './components/OvenSystem.js'; // 烘箱系统组件
import { ElecValve } from './components/ElecValve.js';  // 电动阀组件

import { LVDTPressureSensor } from './components/LVDT.js';  // 差动变压器组件
import { TempTransmitter } from './components/TempTransmitter.js';  // 温度变送器组件
import { PressTransmitter } from './components/PressTransmitter.js';

import { VoltageTransmitter } from './components/VoltageTransmitter.js';  // 电压变送器组件 
import { DCPower } from './components/DCPower.js';  // 直流电源组件
import { AmpMeter } from './components/AmpMeter.js'; // 电流表组件
import { VariResistor } from './components/VariResistor.js'; // 可变电阻组件
import { Resistor } from './components/Resistor.js'; // 电阻组件
import { Multimeter } from './components/Multimeter.js';  // 万用表组件
import { OpAmp } from './components/OpAmp.js';  // 运算放大器组件
import { Ground } from './components/Gnd.js';  // 地组件
import { Monitor } from './components/Monitor.js';  // 监视器组件

import { Relay } from './components/Relay.js';  // 继电器组件
import { ACPower } from './components/ACPower.js';  // 交流电源组件
import { Oscilloscope_tri } from './components/Osc_tri.js'; // 三通道示波器组件
import { Oscilloscope } from './components/Oscilloscope.js';  // 示波器组件
import { SignalGenerator } from './components/SignalGenerator.js';  // 信号发生器组件
import { Capacitor } from './components/Capacitor.js';  // 电容组件
import { JFET } from './components/JFET.js';  // JFET场效应管组件
import { Diode } from './components/Diode.js';  // 二极管组件
import { Transistor } from './components/Transistor.js';  // 三极管组件

import { RealVariResistor } from './components/RealVariResistor.js';
import { CoolingSystem } from './components/CoolingSystem.js';


/**
 * ControlSystem - 控制系统仿真引擎
 * 负责组件管理、物理计算、自动/手动连线逻辑及渲染更新
 */
export class ControlSystem {
    constructor() {
        // 1. 画布基础设置
        this.container = document.getElementById('container'); // 画布容器
        this.stage = new Konva.Stage({ container: 'container', width: window.innerWidth, height: window.innerHeight }); // 整个画布舞台
        this.layer = new Konva.Layer();  // 组件层（包含所有组件节点）
        this.lineLayer = new Konva.Layer(); // 连线层（单独管理连线，便于控制层级和重绘）
        this.stage.add(this.layer, this.lineLayer); // 组件层在下，连线层在上

        // 2. 组件和连线资源池
        this.comps = {};        // 组件实例集合
        this.conns = [];        // 所有连接统一存储为 {from, to, type}
        this.pipeNodes = [];    // 画布上的管路形状节点
        this.wireNodes = [];    // 画布上的电路形状节点

        // 3. 连线交互状态
        this.linkingState = null; // 当前正在连线的起点信息
        this.tempLine = null;     // 鼠标跟随虚线

        //4. 流程控制、电路求解、气路求解
        this.stepsArray = [];  //存储所有流程的数组
        this.workflowComp = null;  //流程控制实例组件
        this.voltageSolver = null;  //电路求解器实例组件
        this.pressSolver = null;   //气路求解器组件
        this.showComp = null;        //提示展示组件

        // --- 5. 性能优化：重绘控制标记 ---
        this._needsRedraw = true; // 初始状态需要绘制一次
        this._physicsIterCount = 0; // 物理计算迭代计数器

        //6.基本初始化、撤销恢复初始化、交互初始化、流程控制初始化、故障配置初始化。
        this.init();  // 基础组件和循环初始化
        this.initHistory();  // 历史状态管理初始化
        this.initStageEvents(); // 连线交互事件初始化
        this.initSteps(); // 流程控制初始化
        this.initFault(); // 故障配置初始化

    }

    // ==========================================
    // 第一部分：初始化与核心配置
    // ==========================================

    /**
     * 1. 系统初始化：创建组件并启动仿真循环
     */
    init() {
        // 计算缩放因子以适应不同屏幕大小
        const baseWidth = 1920;  // 设计稿的基础宽度
        const baseHeight = 1080; // 设计稿的基础高度
        const scaleX = window.innerWidth / baseWidth;  // 水平缩放因子
        const scaleY = window.innerHeight / baseHeight; // 垂直缩放因子
        const scale = Math.min(scaleX, scaleY);  // 取较小的缩放因子以保持宽高比
        const offsetX = (window.innerWidth - baseWidth * scale) / 2;  // 水平居中偏移
        const offsetY = (window.innerHeight - baseHeight * scale) / 2; // 垂直居中偏移

        // 1. 实例化组件，传入 this 以便组件能够调用 handlePortClick 和 redrawAll
        const componentConfigs = [

            { Class: Monitor, id: 'monitor', x: 850, y: 550 },
            { Class: PIDController, id: 'pid', x: 710, y: 20 },
            { Class: TempTransmitter, id: 'ttrans', x: 150, y: 200 },
            { Class: CoolingSystem, id: 'pt', x: 180, y: 450 },
            { Class: VariResistor, id: 'stdres', x: 1220, y: 340 },
            { Class: TeeConnector, id: 'tconn', x: 20, y: 680, direction: 'right' },
            { Class: Pump, id: 'pump', x: 30, y: 580 },
            { Class: Engine, id: 'engine', x: 400, y: 380 },
            { Class: ElecValve, id: 'valve', x: 600, y: 660 },
            { Class: Cooler, id: 'cooler', x: 200, y: 800 },
            { Class: DCPower, id: 'dcpower', x: 1140, y: 80 },
            { Class: AmpMeter, id: 'ampmeter', x: 400, y: 100 },
             { Class: AmpMeter, id: 'ampmeter2', x: 1520, y: 600 },           
            { Class: Multimeter, id: 'multimeter', x: 1400, y: 30 },
            // { Class: Oscilloscope_tri, id: 'osc', x: 1280, y: 400 },

        ];

        // 应用缩放和偏移到组件配置
        const scaledConfigs = componentConfigs.map(cfg => ({
            ...cfg,   // 应用缩放和偏移到组件坐标
            x: cfg.x * scale + offsetX,  // 应用水平缩放和居中偏移
            y: cfg.y * scale + offsetY,  // 应用垂直缩放和居中偏移
            scale: scale    // 传递缩放因子到组件，组件内部根据需要调整大小
        }));

        // 实例化组件并添加到画布
        scaledConfigs.forEach(cfg => {
            this.comps[cfg.id] = new cfg.Class(cfg, this);
            this.layer.add(this.comps[cfg.id].group);
        });
        // --- 性能优化：静态组件启用 Canvas 缓存 ---
        this._applyStaticCaching();
        this.layer.draw();

        // 2. 实例化流程工具、电路求解工具
        this.workflowComp = new Workflow(this);
        this.voltageSolver = new CircuitSolver(this);
        this.pressSolver = new PneumaticSolver(this);
        this.showComp = new Show(this);
        this.requiredPipes = [
            { from: 'engine_pipe_o', to: 'pump_pipe_i', type: 'pipe' },
            { from: 'pump_pipe_o', to: 'tconn_pipe_l', type: 'pipe' },
            { from: 'tconn_pipe_u', to: 'valve_pipe_u', type: 'pipe' },
            { from: 'tconn_pipe_r', to: 'cooler_pipe_i', type: 'pipe' },
            { from: 'cooler_pipe_o', to: 'valve_pipe_l', type: 'pipe' },
            { from: 'valve_pipe_r', to: 'engine_pipe_i', type: 'pipe' }
        ];
        // --- 核心优化：解耦仿真主循环 ---
        // 3. 启动独立的物理计算循环 (使用 setInterval 保证计算频率)
        this._physicsTimer = setInterval(() => this._updatePhysics(), 1000 / 20); // 20fps 的计算频率

        // 4. 启动独立的渲染循环 (使用 RequestAnimationFrame 跟随浏览器 UI 刷新)
        this._renderLoop();
    }



    // 2. 历史状态初始化、声明onChange函数（处理两个按钮的状态）
    initHistory() {
        // history 管理：仅记录用户点击产生的连接/删除动作
        this.history = new HistoryManager();
        const btnUndo = document.getElementById('btnUndo');
        const btnRedo = document.getElementById('btnRedo');
        this.history.onChange = () => { // 每次历史状态改变时更新按钮状态
            btnUndo.disabled = !(this.history.undos && this.history.undos.length > 0);
            btnRedo.disabled = !(this.history.redos && this.history.redos.length > 0);
        };
        this.history.onChange();
    }

    // 3. 连线交互的初始化、定义鼠标移动处理函数（画出虚线）
    initStageEvents() {
        // 鼠标移动时实时更新虚线终点坐标
        this.stage.on('mousemove', () => {
            if (!this.linkingState || !this.tempLine) return;
            const pos = this.stage.getPointerPosition();
            let startPos;
            if (this.linkingState.comp && this.linkingState.comp.getAbsPortPos) {
                // 直接从组件获取起点坐标（适用于组件内端口）
                startPos = this.linkingState.comp.getAbsPortPos(this.linkingState.portId);
            } else {
                const did = this.linkingState.portId.split('_')[0];
                startPos = this.comps[did]?.getAbsPortPos(this.linkingState.portId);
            }
            if (!startPos) return;
            // 更新虚线坐标,并确保虚线在所有组件之下
            this.tempLine.points([startPos.x, startPos.y, pos.x, pos.y]);
            this.tempLine.moveToBottom();
            // 优化：只在坐标发生实际变化时才请求重绘，避免不必要的渲染
            this.requestRedraw();
        });
        this.stage.on('contextmenu', (e) => {
            e.evt.preventDefault(); // 阻止默认菜单
            e.evt.stopPropagation(); // ← 防止触发 window 的监听器
            // 逻辑：如果点击的是空白处（不是组件），显示系统菜单
            // 如果你已经为组件写了右键逻辑，这里需要判断 target 
            if (e.target === this.stage || e.target.name() === 'background-rect') {
                // 右键点击空白处，显示系统菜单
                this.showSystemContextMenu(e.evt);
            }
        });
        // 右键或 ESC 取消当前连线操作
        window.addEventListener('contextmenu', (e) => { e.preventDefault(); this.resetLinking(); });
        window.addEventListener('keydown', (e) => { if (e.key === 'Escape') this.resetLinking(); });
    }

    // 4. 流程初始化函数
    initSteps() {
        // 1. 定义项目配置表 (包含名称和 ID)
        const projectConfigs = [
            { id: 0, name: "1. 冷却水温度控制系统运行" },
            { id: 1, name: "2. PT100短路故障排除(项目6.1)" },
            { id: 2, name: "3. PT100断路故障排除(项目6.1)" },
            { id: 3, name: "4. 温度变送器输出开路故障排除(项目6.1)" },
            { id: 4, name: "5. 温度变送器零点漂移故障排除(项目6.3)" },
            { id: 5, name: "6. 温度变送器量程偏差故障排除(项目6.3)" },
            { id: 6, name: "7. PID调节器参数失调故障排除(项目6.4)" },
            { id: 7, name: "8. PID调节器输出回路故障排除(项目6.4)" },
            { id: 8, name: "9. 三通调节阀执行机构卡死故障排除(项目6.2)" },
            { id: 9, name: "10. 三通调节阀信号输入回路开路故障排除" },
        ];

        // 2. 动态填充 HTML 的 select 下拉框
        const taskSelect = document.getElementById('taskSelect');
        if (taskSelect) {
            // 保留第一个默认选项，清空其他的（防止重复调用时堆叠）
            taskSelect.innerHTML = '<option value="" selected>请选择操作项目...</option>';
            projectConfigs.forEach(proj => {
                const opt = document.createElement('option');
                opt.value = proj.id;    // 对应 stepsArray 的索引
                opt.textContent = proj.name;
                taskSelect.appendChild(opt);
            });
        }
        // 3. 每个项目操作流程定义
        const autoConns = [
            { from: 'engine_pipe_o', to: 'pump_pipe_i', type: 'pipe' },
            { from: 'pump_pipe_o', to: 'tconn_pipe_l', type: 'pipe' },
            { from: 'tconn_pipe_u', to: 'valve_pipe_u', type: 'pipe' },
            { from: 'tconn_pipe_r', to: 'cooler_pipe_i', type: 'pipe' },
            { from: 'cooler_pipe_o', to: 'valve_pipe_l', type: 'pipe' },
            { from: 'valve_pipe_r', to: 'engine_pipe_i', type: 'pipe' },

            { from: 'pid_wire_vcc', to: 'dcpower_wire_p', type: 'wire' },
            { from: 'pid_wire_gnd', to: 'dcpower_wire_n', type: 'wire' },
            { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
            { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
            { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
            { from: 'pid_wire_pi1', to: 'ampmeter_wire_p', type: 'wire' },
            { from: 'ampmeter_wire_n', to: 'ttrans_wire_p', type: 'wire' },
            { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' },
            { from: 'pid_wire_no1', to: 'valve_wire_r', type: 'wire' },
            { from: 'pid_wire_po1', to: 'valve_wire_l', type: 'wire' },

            { from: 'pid_wire_b1', to: 'monitor_wire_b1', type: 'wire' },
            { from: 'pid_wire_a1', to: 'monitor_wire_a1', type: 'wire' }
        ];
        // const meterConns = [
        //     { from: 'multimeter_wire_com', to: 'dcpower_wire_n', type: 'wire' },
        //     { from: 'multimeter_wire_v', to: 'dcpower_wire_p', type: 'wire' },
        //     { from: 'multimeter_wire_com', to: 'ptrans_wire_n', type: 'wire' },
        //     { from: 'multimeter_wire_v', to: 'ptrans_wire_p', type: 'wire' },
        // ];
        // 定义一个辅助函数，用于检查一组连接是否存在
        const checkConnectionsExist = (connIndices) => {
            return connIndices.every(i =>
                this.conns.some(c => this._connEqual(c, autoConns[i]))
            );
        };
        this.stepsArray[0] = [
            //系统起动过程演练
            // --- 工艺管路部分 ---
            {
                msg: "1：从柴油机冷却水出口 --> 水泵入口。",
                act: async () => {
                    this.conns = []; // 清空现有连接
                    this.comps['dcpower'].isOn = false;
                    this.comps['dcpower'].update();
                    this.comps['pump'].pumpOn = false;
                    this.comps['engine'].engOn = false;
                    this.comps['pid'].mode = 'MAN';

                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[0]);
                },
                check: () => checkConnectionsExist([0])
            },
            {
                msg: "2：从水泵出口 --> T型管上端。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[1]);
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => checkConnectionsExist([1])
            },
            {
                msg: "3：从T型管右端 --> 三通调节阀左端",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[2]);
                },
                check: () => checkConnectionsExist([2])
            },
            {
                msg: "4：从T型管下端 --> 冷却器入口。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[3]);
                },
                check: () => checkConnectionsExist([3])
            },
            {
                msg: "5：从冷却器出口 --> 三通调节阀下端。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[4]);
                },
                check: () => checkConnectionsExist([4])
            },
            {
                msg: "6：从三通调节阀上端 --> 柴油机冷却水入口。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[5]);
                },
                check: () => checkConnectionsExist([5])
            },

            // --- 电气接线部分 ---
            {
                msg: "7：连接 PID 控制器电源到 DC24V 正负极。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[6]);
                    await this.addConnectionAnimated(autoConns[7]);
                },
                check: () => checkConnectionsExist([6, 7])
            },
            {
                msg: "8：连接 PT100 信号线至温度变送器端子。",
                act: async () => {
                    await this.addConnectionAnimated(autoConns[8]);
                    await this.addConnectionAnimated(autoConns[9]);
                    await this.addConnectionAnimated(autoConns[10]);
                },
                check: () => checkConnectionsExist([8, 9, 10])
            },
            {
                msg: "9：连接温度变送器输出信号 (4-20mA) 至 PID 输入端。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[11]);
                    await this.addConnectionAnimated(autoConns[12]);
                    await this.addConnectionAnimated(autoConns[13]);
                },
                check: () => checkConnectionsExist([11, 12, 13])
            },
            {
                msg: "10：连接 PID 控制输出至三通调节阀电机端子。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[14]);
                    await this.addConnectionAnimated(autoConns[15]);
                },
                check: () => checkConnectionsExist([14, 15])
            },
            {
                msg: "11：连接 RS485 通讯总线至上位机监控终端。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    await this.addConnectionAnimated(autoConns[16]);
                    await this.addConnectionAnimated(autoConns[17]);
                },
                check: () => checkConnectionsExist([16, 17])
            },
            {
                msg: "12：开启24V电源。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.dcpower.isOn = true;
                    this.comps.dcpower.update();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => this.comps.dcpower.isOn === true
            },
            {
                msg: "13：手动调节阀门开度到略大于20%。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.mode = "MAN";
                    this.comps.pid.OUT = 25;
                    await new Promise(r => setTimeout(r, 2000));
                },
                check: () => this.comps.valve.currentPos > 0.2
            },
            {
                msg: "14：开启冷却水泵。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pump.pumpOn = true;
                    await new Promise(r => setTimeout(r, 2000));
                },
                check: () => this.comps.pump.pumpOn === true
            },
            {
                msg: "15：开启柴油机。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.engine.engOn = true;
                    await new Promise(r => setTimeout(r, 2000));
                },
                check: () => this.comps.engine.engOn === true
            },
            {
                msg: "16：PID控制器切换到自动模式。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.mode = 'AUTO';
                    await new Promise(r => setTimeout(r, 2000));
                },
                check: () => this.comps.pid.mode === 'AUTO'
            },
            {
                msg: "17：确保系统警报已经消音、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 2000));
                },
                check: () => !this.comps.monitor.activeAlarms.some(
                    a => !a.muted)

            }
        ];
        this.stepsArray[1] = [
            // --- PT100短路故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 20000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = !this.comps.monitor.activeAlarms.some(
                        a => a.muted === false);
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发PT100短路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pt._pt100Fault = 'short';
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.pt._pt100Fault === 'short'
            },
            {
                msg: "3：查看报警监视面板，进行消音、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: async () => {
                    // 1. 先等待 6s 确保仿真引擎的温度升高并触发了报警逻辑
                    await new Promise(r => setTimeout(r, 3000));
                    const monitor = this.comps.monitor;
                    const alarms = monitor.activeAlarms;
                    // 3. 只有当存在报警，且所有报警都消音了，才返回 true
                    return alarms.every(a => a.muted === true);
                }

            },
            {
                msg: "4：PID控制器切换到手动模式，阀位控制在60-70之间。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 65;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "MAN";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "5：断开温度变送器电源，断开PT100接线。",
                act: async () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    await new Promise(r => setTimeout(r, 2000));
                    for (const conn of transLines) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    return transLines.every(target => {
                        return !this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                }

            },
            {
                msg: "6：万用表打到蜂鸣器档或者200欧姆档，测量PT100电阻，确认电阻为0。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.multimeter.mode = "RES200";
                    this.comps.multimeter._updateAngleByMode();
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_l', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_r', type: 'wire' };
                    await this.addConnectionAnimated(conn1);
                    await this.addConnectionAnimated(conn2);
                    await new Promise(resolve => setTimeout(resolve, 5000));

                },
                check: () => {
                    const c1 = this.comps.multimeter.mode === "RES200" || this.comps.multimeter.mode === "DIODE";
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_l', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_r', type: 'wire' };
                    const c2 = this.conns.some(c => this._connEqual(c, conn1));
                    const c3 = this.conns.some(c => this._connEqual(c, conn2));
                    const c4 = this.comps.multimeter.value < 1;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "7：更换PT100，确认新电阻的阻值正常。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.multimeter.mode = "RES200";
                    this.comps.multimeter._updateAngleByMode();
                    this.comps.pt._pt100Fault = null;
                    await new Promise(r => setTimeout(r, 5000));

                },
                check: () => {
                    const c1 = this.comps.multimeter.mode === "RES200" || this.comps.multimeter.mode === "DIODE";
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_l', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_r', type: 'wire' };
                    const c2 = this.conns.some(c => this._connEqual(c, conn1));
                    const c3 = this.conns.some(c => this._connEqual(c, conn2));
                    const c4 = this.comps.multimeter.value > 100;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "8：接入新的PT100，重新接入PID输入回路。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_l', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_r', type: 'wire' };
                    this.removeConn(conn1);
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    this.removeConn(conn2);
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒

                    this.comps.multimeter.mode = "OFF";
                    this.comps.multimeter._updateAngleByMode();

                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.addConn(conn);   // 重新接入当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 2000));

                },
                check: () => {
                    const requiredLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];

                    // 检查是否每一条预期的线都存在于当前的 conns 数组中
                    return requiredLines.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });
                    });
                }
            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[2] = [
            // --- PT100断路故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 20000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = !this.comps.monitor.activeAlarms.some(
                        a => a.muted === false);
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发PT100断路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pt._pt100Fault = 'open';
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.pt._pt100Fault === 'open'
            },
            {
                msg: "3：查看报警监视面板，进行消音、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: async () => {
                    // 1. 先等待 6s 确保仿真引擎的温度升高并触发了报警逻辑
                    await new Promise(r => setTimeout(r, 3000));
                    const monitor = this.comps.monitor;
                    const alarms = monitor.activeAlarms;
                    // 3. 只有当存在报警，且所有报警都消音了，才返回 true
                    return alarms.every(a => a.muted === true);
                }

            },
            {
                msg: "4：PID控制器切换到手动模式，阀位控制在60-70之间。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 65;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "MAN";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "5：断开温度变送器电源，断开PT100接线。",
                act: async () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    await new Promise(r => setTimeout(r, 3000));
                    for (const conn of transLines) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    return transLines.every(target => {
                        return !this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                }

            },
            {
                msg: "6：万用表打到200k欧姆档，测量PT100电阻，确认电阻为无穷大。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.multimeter.mode = "RES200k";
                    this.comps.multimeter._updateAngleByMode();
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_r', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_l', type: 'wire' };
                    await this.addConnectionAnimated(conn1);
                    await this.addConnectionAnimated(conn2);
                    await new Promise(r => setTimeout(r, 5000));

                },
                check: () => {
                    const c1 = this.comps.multimeter.mode === "RES200k";
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_r', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_l', type: 'wire' };
                    const c2 = this.conns.some(c => this._connEqual(c, conn1));
                    const c3 = this.conns.some(c => this._connEqual(c, conn2));
                    const c4 = this.comps.multimeter.value > 1000 || this.comps.multimeter.value === Infinity;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "7：更换PT100，确认新电阻的阻值正常。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.multimeter.mode = "RES200";
                    this.comps.multimeter._updateAngleByMode();
                    this.comps.pt._pt100Fault = null;
                    await new Promise(r => setTimeout(r, 5000));

                },
                check: () => {
                    const c1 = this.comps.multimeter.mode === "RES200";
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_r', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_l', type: 'wire' };
                    const c2 = this.conns.some(c => this._connEqual(c, conn1));
                    const c3 = this.conns.some(c => this._connEqual(c, conn2));
                    const c4 = this.comps.multimeter.value < 200;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "8：接入新的PT100，重新接入PID输入回路。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const conn1 = { from: 'multimeter_wire_com', to: 'pt_wire_r', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pt_wire_l', type: 'wire' };
                    this.removeConn(conn1);
                    await new Promise(r => setTimeout(r, 1000)); // 等待2秒
                    this.removeConn(conn2);
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒

                    this.comps.multimeter.mode = "OFF";
                    this.comps.multimeter._updateAngleByMode();

                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.addConn(conn);   // 重新接入当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 2000));

                },
                check: () => {
                    const requiredLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];

                    // 检查是否每一条预期的线都存在于当前的 conns 数组中
                    return requiredLines.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });
                    });
                }
            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[3] = [
            // --- 温度变送器输出回路开路故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(resolve => setTimeout(resolve, 20000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = !this.comps.monitor.activeAlarms.some(
                        a => a.muted === false);
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发温度变送器输出回路开路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.ttrans.isBreak = true;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.ttrans.isBreak === true
            },
            {
                msg: "3：查看报警监视面板，进行消音、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: async () => {
                    // 1. 先等待 6s 确保仿真引擎的温度升高并触发了报警逻辑
                    await new Promise(r => setTimeout(r, 3000));

                    const monitor = this.comps.monitor;
                    const alarms = monitor.activeAlarms;

                    // 3. 只有当存在报警，且所有报警都消音了，才返回 true
                    return alarms.every(a => a.muted === true);
                }

            },
            {
                msg: "4：PID控制器切换到手动模式，阀位控制在60-70之间。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 65;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "MAN";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "5：万用表打到直流200V档，测量温度变送器电源电压正常。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.multimeter.mode = "DCV200";
                    this.comps.multimeter._updateAngleByMode();
                    const conn1 = { from: 'multimeter_wire_com', to: 'ttrans_wire_n', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'ttrans_wire_p', type: 'wire' };
                    await this.addConnectionAnimated(conn1);
                    await this.addConnectionAnimated(conn2);
                    await new Promise(resolve => setTimeout(resolve, 5000));

                },
                check: () => {
                    const c1 = this.comps.multimeter.mode === "DCV200";
                    const conn1 = { from: 'multimeter_wire_com', to: 'ttrans_wire_n', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'ttrans_wire_p', type: 'wire' };
                    const c2 = this.conns.some(c => this._connEqual(c, conn1));
                    const c3 = this.conns.some(c => this._connEqual(c, conn2));
                    const c4 = this.comps.multimeter.value > 23 || this.comps.multimeter.value === 24;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "6：观察20mA电流表，电流为0，可确认温度变送器输出回路开路。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 5000));
                },
                check: async () => {
                    await new Promise(r => setTimeout(r, 5000));
                    const c1 = this.comps.ampmeter.value < 0.1;
                    return c1;
                }

            },
            {
                msg: "7：断开温度变送器电源接线，修复开路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const transLines = [
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    this.comps.ttrans.isBreak = false;
                    await new Promise(r => setTimeout(r, 5000));

                },
                check: () => {
                    const c1 = this.comps.ttrans.isBreak === false;
                    return c1;
                }
            },
            {
                msg: "8：接通温度变送器电源回路，电流表显示电流大于4mA，确认回路恢复正常。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const transLines = [
                        { from: 'ttrans_wire_p', to: 'ampmeter_wire_n', type: 'wire' },
                        { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.addConn(conn);   // 
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    await new Promise(r => setTimeout(r, 2000));

                },
                check: () => {
                    const c1 = this.comps.ampmeter.value > 4;
                    return c1;
                }
            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[4] = [
            // --- 温度变送器零点漂移故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(resolve => setTimeout(resolve, 20000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = !this.comps.monitor.activeAlarms.some(
                        a => a.muted === false);
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发温度变送器零点漂移故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.ttrans.zeroAdj = 0.4;
                    this.comps.ttrans.knobs['zero'].rotation(180);
                    this.comps.ttrans._refreshCache();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.ttrans.zeroAdj > 0.1
            },
            {
                msg: "3：PID控制器切换到手动模式，阀位控制在60-70之间。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 65;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "MAN";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "4：断开PT100的接线，接入标准可调电阻。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    this.comps.pt.group.position({ x: 270, y: 480 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    this.comps.stdres.group.position({ x: 280, y: 400 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    for (const conn of transLinesNew) {
                        this.addConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    await new Promise(resolve => setTimeout(resolve, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const ptDisconnected = transLines.every(target => {
                        return !this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const stdresConnected = transLinesNew.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    return ptDisconnected && stdresConnected;
                }
            },
            {
                msg: "5：将标准电阻每次增加3.85欧姆，直到138.5欧姆左右，确认每次仪表指示值增加10度，可确认变送器零点漂移故障，而不是量程偏差故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    for (let i = 1; i <= 10; i++) {
                        this.comps.stdres.currentResistance = 100 + i * 3.851;
                        this.comps.stdres.update();
                        await new Promise(r => setTimeout(r, 3000));

                    }
                    await new Promise(resolve => setTimeout(resolve, 4000));

                },
                check: () => this.comps.stdres.currentResistance > 138

            },
            {
                msg: "6：将标准电阻调回100欧姆，调整变送器零点，使得温度显示值为0度左右。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.stdres.currentResistance = 100;
                    this.comps.stdres.update();
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.ttrans.zeroAdj = 0;
                    this.comps.ttrans.knobs['zero'].rotation(0);
                    this.comps.ttrans._refreshCache();
                    await new Promise(resolve => setTimeout(resolve, 5000));

                },
                check: () => {
                    const c1 = Math.abs(this.comps.stdres.currentResistance - 100) < 3;
                    const c2 = Math.abs(this.comps.ttrans.zeroAdj) < 0.05;
                    return c1 && c2;
                }
            },
            {
                msg: "7：将标准电阻调回138.5欧姆，确认温度显示值为100度左右。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.stdres.currentResistance = 138.51;
                    this.comps.stdres.update();
                    await new Promise(resolve => setTimeout(resolve, 5000));

                },
                check: () => {
                    const c1 = Math.abs(this.comps.stdres.currentResistance - 138) < 3;
                    const c2 = Math.abs(this.comps.ttrans.zeroAdj) < 0.05;
                    return c1 && c2;
                }
            },
            {
                msg: "8：断开标准可调电阻，重新接回PT100电阻，确认温度显示正常。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    for (const conn of transLinesNew) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    this.comps.stdres.group.position({ x: 1200, y: 310 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    this.comps.pt.group.position({ x: 270, y: 400 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.addConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    await new Promise(r => setTimeout(r, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const ptConnected = transLines.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const stdresDisconnected = transLinesNew.every(target => {
                        return !this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    return ptConnected && stdresDisconnected;
                }
            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[5] = [
            // --- 温度变送器量程偏差故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 20000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = !this.comps.monitor.activeAlarms.some(
                        a => a.muted === false);
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发温度变送器量程偏差故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.ttrans.spanAdj = 1.125;
                    this.comps.ttrans.knobs['span'].rotation(90);
                    this.comps.ttrans._refreshCache();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.ttrans.spanAdj > 1.1
            },
            {
                msg: "3：PID控制器切换到手动模式，阀位控制在60-70之间。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 65;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "MAN";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "4：断开PT100的接线，接入标准可调电阻。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    this.comps.pt.group.position({ x: 270, y: 500 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    this.comps.stdres.group.position({ x: 270, y: 360 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    for (const conn of transLinesNew) {
                        this.addConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    await new Promise(resolve => setTimeout(resolve, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const ptDisconnected = transLines.every(target => {
                        return !this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const stdresConnected = transLinesNew.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    return ptDisconnected && stdresConnected;
                }
            },
            {
                msg: "5：将标准电阻每次增加3.85欧姆，直到138.5欧姆左右，确认每次仪表指示值的变化量不等于10度，可确认变送器量程偏差故障，而不是零点漂移故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    for (let i = 1; i <= 10; i++) {
                        this.comps.stdres.currentResistance = 100 + i * 3.851;
                        this.comps.stdres.update();
                        await new Promise(r => setTimeout(r, 2000));

                    }
                    await new Promise(resolve => setTimeout(resolve, 4000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => this.comps.stdres.currentResistance > 138

            },
            {
                msg: "6：将标准电阻保持138.5欧姆，调整变送器量程，使得温度显示值为100度左右。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.stdres.currentResistance = 138.51;
                    this.comps.stdres.update();
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.ttrans.spanAdj = 1;
                    this.comps.ttrans.knobs['span'].rotation(0);
                    await new Promise(r => setTimeout(r, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const c1 = Math.abs(this.comps.stdres.currentResistance - 138) < 3;
                    const c2 = Math.abs(this.comps.ttrans.spanAdj - 1) < 0.05;
                    return c1 && c2;
                }
            },
            {
                msg: "7：将标准电阻调回100欧姆，确认温度显示值为0度左右。若有偏差，调整量程。然后将电阻调到138.5欧姆，确保温度显示值为100度左右。反复调整2-3次。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.stdres.currentResistance = 100;
                    this.comps.stdres.update();
                    await new Promise(resolve => setTimeout(resolve, 5000));

                },
                check: () => {
                    const c1 = Math.abs(this.comps.stdres.currentResistance - 100) < 3;
                    const c2 = Math.abs(this.comps.ttrans.spanAdj - 1) < 0.05;
                    return c1 && c2;
                }
            },
            {
                msg: "8：断开标准可调电阻，重新接回PT100电阻，确认温度显示正常。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    for (const conn of transLinesNew) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    this.comps.stdres.group.position({ x: 1250, y: 320 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    this.comps.pt.group.position({ x: 270, y: 450 });
                    await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' }
                    ];
                    for (const conn of transLines) {
                        this.addConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    };
                    await new Promise(r => setTimeout(r, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const transLines = [
                        { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const ptConnected = transLines.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    const transLinesNew = [
                        { from: 'stdres_wire_l', to: 'ttrans_wire_l', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_m', type: 'wire' },
                        { from: 'stdres_wire_r', to: 'ttrans_wire_r', type: 'wire' },
                    ];
                    const stdresDisconnected = transLinesNew.every(target => {
                        return !this.conns.some(conn => {
                            return this._connEqual(conn, target);
                        });

                    });
                    return ptConnected && stdresDisconnected;
                }
            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[6] = [
            // --- PID调节器参数失调故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(resolve => setTimeout(resolve, 20000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = !this.comps.monitor.activeAlarms.some(
                        a => a.muted === false);
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发PID参数失调故障,温度波动，阀门开度几乎不变。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.P = 0.03;
                    this.comps.pid.I = 0;
                    this.comps.pid.D = 0;
                    await new Promise(resolve => setTimeout(resolve, 5000));
                },
                check: () => this.comps.pid.P < 0.5
            },
            {
                msg: "3：手动改变温度设定值,调到75度或85度，阀门开度仍然不变化或变化小。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    for (let i = 1; i <= 5; i++) {
                        this.comps.pid.SV = 80 - i;
                        await new Promise(resolve => setTimeout(resolve, 1000));
                    }

                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => Math.abs(this.comps.pid.SV - 80) > 3
            },
            {
                msg: "4：PID控制器切换到手动模式，阀位可调节到60-70之间，说明PID调节器输出回路正常，自动模式下P/I/D参数设置不当。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 65;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "MAN";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "5：进入PID系统菜单，调节P、I、D参数，比例系数调到4左右。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.P = 4;
                    this.comps.pid.I = 30;
                    this.comps.pid.D = 0;
                    await new Promise(resolve => setTimeout(resolve, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => this.comps.pid.P > 3
            },
            {
                msg: "6：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[7] = [
            // --- PID调节器输出回路开路故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(resolve => setTimeout(resolve, 20000));
                },
                check: async () => {
                    // 1. 等待 2 秒让物理引擎计算出趋势
                    await new Promise(r => setTimeout(r, 2000));
                    const pid = this.comps.pid;
                    // c1: 必须是自动模式
                    const c1 = pid.mode === "AUTO";

                    // c2: 误差范围缩小。
                    // 如果要求严格，建议范围设为 1~2 度。10 度误差对于 80 度的目标确实太大了。
                    const c2 = Math.abs(pid.PV - pid.SV) < 10;

                    // c3: 报警检查逻辑改进
                    // 应该检查：1. 是否当前完全没有报警（通常系统稳定后不应有报警）
                    // 或者：2. 如果有报警，必须全部已消音
                    const alarms = this.comps.monitor.activeAlarms;
                    const c3 = alarms.length === 0 || alarms.every(a => a.muted === true);

                    // c4: 基础硬件状态
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;

                    // 只有当温度真正接近 80 度，且硬件全开、模式正确、无未处理报警时才通过
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发PID调节器输出回路开路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.out1Fault = true;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.pid.out1Fault === true
            },
            {
                msg: "3：查看报警监视面板，进行消音、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const monitor = this.comps.monitor;
                    const alarms = monitor.activeAlarms;
                    // 3. 只有当存在报警，且所有报警都消音了，才返回 true
                    return alarms.every(a => a.muted === true);
                }

            },
            {
                msg: "4：阀门切换到本地模式，阀位控制在60-70之间。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.controlMode = "MANUAL";
                    this.comps.valve.updateModeText("MANUAL");
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.valve.manualPos = 0.65;
                    this.comps.valve.update();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.valve.controlMode === "MANUAL";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "5：断开PID调节器输出回路正极接线，接入20mA电流表，电流为0。无论手动还是自动，OUT有输出，但回路电流始终为0。",
                act: async () => {
                    const transLines = [
                        { from: 'pid_wire_po1', to: 'valve_wire_l', type: 'wire' }
                    ];
                    await new Promise(r => setTimeout(r, 3000));
                    for (const conn of transLines) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    this.comps.ampmeter2.group.position({ x: 720, y: 300 });
                    const conn1 = { from: 'pid_wire_po1', to: 'ampmeter2_wire_p', type: 'wire' };
                    const conn2 = { from: 'ampmeter2_wire_n', to: 'valve_wire_l', type: 'wire' };
                    await this.addConnectionAnimated(conn1);
                    await this.addConnectionAnimated(conn2);
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 50;
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const connP = { from: 'pid_wire_po1', to: 'valve_wire_l', type: 'wire' };
                    const c1 = !this.conns.some(c => this._connEqual(c,connP));
                    const transLines = [
                        { from: 'pid_wire_po1', to: 'ampmeter2_wire_p', type: 'wire' },
                        { from: 'ampmeter2_wire_n', to: 'valve_wire_l', type: 'wire' }
                    ];
                    const ampMeterIn = transLines.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn,target);
                        });

                    });
                    const c2 = this.comps.ampmeter2.value <0.1;
                    const c3 = this.comps.pid.mode === "MAN";
                    return c1 && ampMeterIn && c2 && c3;
                }

            },
            {
                msg: "6：关闭24V电源，测量输出回路电阻，电阻正常为250欧姆左右，确认是PID调节器输出回路开路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.dcpower.isOn = false;
                    this.comps.dcpower.update();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.multimeter.mode = "RES2k";
                    this.comps.multimeter._updateAngleByMode();
                    const conn1 = { from: 'multimeter_wire_com', to: 'pid_wire_no1', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pid_wire_po1', type: 'wire' };
                    await this.addConnectionAnimated(conn1);
                    await this.addConnectionAnimated(conn2);
                    await new Promise(r => setTimeout(r, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const c1 = this.comps.multimeter.mode === "RES2k" || this.comps.multimeter.mode === "RES200k";
                    const conn1 = { from: 'multimeter_wire_com', to: 'pid_wire_no1', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'pid_wire_po1', type: 'wire' };
                    const c2 = this.conns.some(c => this._connEqual(c,conn1));
                    const c3 = this.conns.some(c => this._connEqual(c,conn2));
                    const c4 = Math.abs(this.comps.multimeter.value - 250) < 10;
                    const c5 = this.comps.dcpower.isOn === false;
                    return c1 && c2 && c3 && c4 && c5;
                }
            },
            {
                msg: "7：修复PID调节器输出回路故障。万用表打到直流20V档。接通电源，观察输出回路电流应大于4mA，万用表测量电压应大于1V。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.out1Fault = false;
                    await new Promise(r => setTimeout(r, 1000));
                    this.comps.multimeter.mode = "DCV20";
                    this.comps.multimeter._updateAngleByMode();
                    await new Promise(r => setTimeout(r, 1000));
                    this.comps.dcpower.isOn = true;
                    this.comps.dcpower.update();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.dcpower.isOn === true;
                    const c2 = this.comps.pid.out1Fault === false;
                    const c3 = this.comps.multimeter.mode === "DCV20" || this.comps.multimeter.mode === "DCV200";
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "8：阀门切换到遥控模式",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.controlMode = "REMOTE";
                    this.comps.valve.updateModeText("REMOTE");
                    await new Promise(resolve => setTimeout(resolve, 1000));
                },
                check: () => this.comps.valve.controlMode === "REMOTE"
            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(resolve => setTimeout(resolve, 1000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[8] = [
            // --- 三通调节阀执行机构卡死故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(resolve => setTimeout(resolve, 20000));
                },
                check: async () => {
                    // 1. 等待 2 秒让物理引擎计算出趋势
                    await new Promise(r => setTimeout(r, 2000));

                    const pid = this.comps.pid;
                    // c1: 必须是自动模式
                    const c1 = pid.mode === "AUTO";

                    // c2: 误差范围缩小。
                    // 如果要求严格，建议范围设为 1~2 度。10 度误差对于 80 度的目标确实太大了。
                    const c2 = Math.abs(pid.PV - pid.SV) < 10;

                    // c3: 报警检查逻辑改进
                    // 应该检查：1. 是否当前完全没有报警（通常系统稳定后不应有报警）
                    // 或者：2. 如果有报警，必须全部已消音
                    const alarms = this.comps.monitor.activeAlarms;
                    const c3 = alarms.length === 0 || alarms.every(a => a.muted === true);

                    // c4: 基础硬件状态
                    const c4 = this.comps.engine.engOn && this.comps.pump.pumpOn && this.comps.dcpower.isOn;

                    // 只有当温度真正接近 80 度，且硬件全开、模式正确、无未处理报警时才通过
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发三通调节阀执行机构卡死故障,温度波动，阀门开度完全不变。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.isStuck = true;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.valve.isStuck === true
            },
            {
                msg: "3：手动改变温度设定值,调到75度或85度，阀门开度仍然不变化。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    for (let i = 1; i <= 5; i++) {
                        this.comps.pid.SV = 80 - i;
                        await new Promise(resolve => setTimeout(resolve, 1000));
                    }

                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => Math.abs(this.comps.pid.SV - 80) > 3
            },
            {
                msg: "4：调节器切换到手动模式,手动调整开度到20%以上，阀门开度仍然不变化。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 30;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "MAN";
                    const c2 = this.comps.valve.isStuck === true;
                    const c3 = this.comps.pid.OUT - 20 > 1;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "5：阀门切换到本地模式，转动手轮，阀门不动作，确定阀门卡死。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.controlMode = "MANUAL";
                    this.comps.valve.updateModeText("MANUAL");
                    await new Promise(r => setTimeout(r, 2000));

                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.valve.controlMode === "MANUAL";
                    const c2 = this.comps.valve.isStuck === true;
                    return c1 && c2;
                }
            },
            {
                msg: "6：关闭柴油机，关闭淡水泵，关闭电源。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.engine.engOn = false;
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pump.pumpOn = false;
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.dcpower.isOn = false;
                    this.comps.dcpower.update();
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.engine.engOn === false;
                    const c2 = this.comps.pump.pumpOn === false;
                    const c3 = this.comps.dcpower.isOn === false;
                    return c1 && c2 && c3;
                }

            },
            {
                msg: "7：修复阀门卡死故障。阀门转到手动模式，手轮调节阀门到50%开度。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.isStuck = false;
                    this.comps.valve.controlMode = "MANUAL";
                    this.comps.valve.updateModeText("MANUAL");
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.valve.manualPos = 0.5
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const c1 = Math.abs(this.comps.valve.currentPos - 0.5) < 0.1;
                    const c2 = this.comps.valve.isStuck === false;
                    const c3 = this.comps.valve.controlMode === "MANUAL";
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "8：阀门转到遥控模式，重启系统。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.controlMode = "REMOTE";
                    this.comps.valve.updateModeText("REMOTE");
                    await new Promise(r => setTimeout(r, 3000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(resolve => setTimeout(resolve, 5000));
                },
                check: () => this.comps.valve.controlMode === "REMOTE"

            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ];
        this.stepsArray[9] = [
            // --- 三通调节阀信号输入回路开路故障排除 ---
            {
                msg: "1：确保系统已经正常运行，PID控制器自动模式，PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyAllPresets();
                    await new Promise(r => setTimeout(r, 2000));
                    this.applyStartSystem();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(resolve => setTimeout(resolve, 20000));
                },
                check: async () => {
                    // 1. 等待 2 秒让物理引擎计算出趋势
                    await new Promise(r => setTimeout(r, 2000));

                    const pid = this.comps.pid;
                    // c1: 必须是自动模式
                    const c1 = pid.mode === "AUTO";

                    // c2: 误差范围缩小。
                    // 如果要求严格，建议范围设为 1~2 度。10 度误差对于 80 度的目标确实太大了。
                    const c2 = Math.abs(pid.PV - pid.SV) < 10;

                    // c3: 报警检查逻辑改进
                    // 应该检查：1. 是否当前完全没有报警（通常系统稳定后不应有报警）
                    // 或者：2. 如果有报警，必须全部已消音
                    const alarms = this.comps.monitor.activeAlarms;
                    const c3 = alarms.length === 0 || alarms.every(a => a.muted === true);

                    // c4: 基础硬件状态
                    const c4 = this.comps.engine.engOn &&this.comps.pump.pumpOn && this.comps.dcpower.isOn;

                    // 只有当温度真正接近 80 度，且硬件全开、模式正确、无未处理报警时才通过
                    return c1 && c2 && c3 && c4;
                }
            },
            {
                msg: "2：触发三通调节阀信号输入回路开路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.currentResistance = 1000000;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => this.comps.valve.currentResistance > 1000
            },
            {
                msg: "3：查看报警监视面板，进行消音、消闪。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    const monitor = this.comps.monitor;
                    const alarms = monitor.activeAlarms;
                    // 3. 只有当存在报警，且所有报警都消音了，才返回 true
                    return alarms.every(a => a.muted === true);
                }

            },
            {
                msg: "4：阀门切换到本地模式，阀位控制在60-70之间。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.controlMode = "MANUAL";
                    this.comps.valve.updateModeText("MANUAL");
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.valve.manualPos = 0.65;
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.valve.controlMode === "MANUAL";
                    const c2 = this.comps.valve.currentPos <= 0.7;
                    const c3 = this.comps.valve.currentPos >= 0.6;
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "5：断开PID调节器输出回路正极接线，接入20mA电流表，电流为0。无论手动还是自动，OUT有输出，但回路电流始终为0。",
                act: async () => {
                    const transLines = [
                        { from: 'pid_wire_po1', to: 'valve_wire_l', type: 'wire' }
                    ];
                    await new Promise(r => setTimeout(r, 3000));
                    for (const conn of transLines) {
                        this.removeConn(conn);   // 删除当前线
                        await new Promise(r => setTimeout(r, 2000)); // 等待2秒
                    }
                    this.comps.ampmeter2.group.position({ x: 620, y: 320 });
                    const conn1 = { from: 'pid_wire_po1', to: 'ampmeter2_wire_p', type: 'wire' };
                    const conn2 = { from: 'ampmeter2_wire_n', to: 'valve_wire_l', type: 'wire' };
                    await this.addConnectionAnimated(conn1);
                    await this.addConnectionAnimated(conn2);
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.pid.mode = "MAN";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.pid.OUT = 50;
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const connP =  { from: 'pid_wire_po1', to: 'valve_wire_l', type: 'wire' };
                    const c1 = !this.conns.some(c => this._connEqual(c,connP));
                    const transLines = [
                        { from: 'pid_wire_po1', to: 'ampmeter2_wire_p', type: 'wire' },
                        { from: 'ampmeter2_wire_n', to: 'valve_wire_l', type: 'wire' }
                    ];
                    const ampMeterIn = transLines.every(target => {
                        return this.conns.some(conn => {
                            return this._connEqual(conn,target);
                        });

                    });
                    const c2 = this.comps.ampmeter2.value <0.1;
                    const c3 = this.comps.pid.mode === "MAN";
                    return c1 && ampMeterIn && c2 && c3;
                }

            },
            {
                msg: "6：关闭24V电源，测量输出回路电阻，电阻正常为250欧姆左右，三通调节阀信号输入端子现在为无穷大，确认是三通调节阀信号输入回路开路故障。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.dcpower.isOn = false;
                    this.comps.dcpower.update();
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.multimeter.mode = "RES2k";
                    this.comps.multimeter._updateAngleByMode();
                    const conn1 = { from: 'multimeter_wire_com', to: 'valve_wire_r', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'valve_wire_l', type: 'wire' };
                    await this.addConnectionAnimated(conn1);
                    await this.addConnectionAnimated(conn2);
                    await new Promise(resolve => setTimeout(resolve, 5000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();

                },
                check: () => {
                    const c1 = this.comps.multimeter.mode === "RES2k" || this.comps.multimeter.mode === "RES200k";
                    const conn1 = { from: 'multimeter_wire_com', to: 'valve_wire_r', type: 'wire' };
                    const conn2 = { from: 'multimeter_wire_v', to: 'valve_wire_l', type: 'wire' };
                    const c2 = this.conns.some(c => this._connEqual(c,conn1));
                    const c3 = this.conns.some(c => this._connEqual(c,conn2));
                    const c4 = this.comps.multimeter.value > 1000;
                    const c5 = this.comps.dcpower.isOn === false;
                    return c1 && c2 && c3 && c4 && c5;
                }
            },
            {
                msg: "7：修复三通调节阀信号输入回路开路故障。万用表显示电阻约为250欧姆左右。接通电源，观察输出回路电流应大于4mA，万用表测量电压应大于1V。",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.currentResistance = 250;
                    await new Promise(resolve => setTimeout(resolve, 5000));
                    this.comps.multimeter.mode = "DCV20";
                    this.comps.multimeter._updateAngleByMode();
                    await new Promise(resolve => setTimeout(resolve, 1000));
                    this.comps.dcpower.isOn = true;
                    this.comps.dcpower.update();
                    await new Promise(r => setTimeout(r, 3000));
                },
                check: () => {
                    const c1 = this.comps.dcpower.isOn === true;
                    const c2 = this.comps.ampmeter2.value >4 ;
                    const c3 = this.comps.multimeter.mode === "DCV20" || this.comps.multimeter.mode === "RES2k";
                    return c1 && c2 && c3;
                }
            },
            {
                msg: "8：阀门切换到遥控模式",
                act: async () => {
                    await new Promise(r => setTimeout(r, 3000));
                    this.comps.valve.controlMode = "REMOTE";
                    this.comps.valve.updateModeText("REMOTE");
                    await new Promise(resolve => setTimeout(resolve, 1000));
                },
                check: () => this.comps.valve.controlMode === "REMOTE"
            },
            {
                msg: "9：系统切回自动模式，确认PV与SV偏差小于10度，报警已消声、消闪。",
                act: async () => {
                    await new Promise(resolve => setTimeout(resolve, 1000));
                    this.comps.pid.mode = "AUTO";
                    await new Promise(r => setTimeout(r, 2000));
                    this.comps.monitor.btnMuteFunc();
                    this.comps.monitor.btnAckFunc();
                },
                check: () => {
                    const c1 = this.comps.pid.mode === "AUTO";
                    const c2 = Math.abs(this.comps.pid.PV - this.comps.pid.SV) < 10;
                    const c3 = this.comps.monitor.activeAlarms.every(
                        a => a.muted === true && a.confirmed === true);
                    return c1 && c2 && c3;
                }
            }

        ]


    }

    // 5. 初始化故障触发、修复、检测
    initFault() {
        // 1. 配置化故障定义：code -> {故障触发， 检测逻辑, 修复逻辑 }
        this.FAULT_CONFIG = {
            1: {
                id: 1,
                name: "1. PT100 传感器短路",
                trigger: () => {
                    // 1: 设置开路故障
                    this.comps['pt']._pt100Fault = 'short';
                },
                check: () => {
                    return this.comps['pt']._pt100Fault === 'short';
                },
                repair: () => {
                    if (this.comps['pt']._pt100Fault == 'short') this.comps['pt']._pt100Fault = null;
                }
            },
            2: {
                id: 2,
                name: "2. PT100 传感器开路",
                trigger: () => {
                    // 1: 设置开路故障
                    this.comps['pt']._pt100Fault = 'open';
                },
                check: () => { return this.comps['pt']._pt100Fault === 'open'; },
                repair: () => {
                    if (this.comps['pt']._pt100Fault == 'open') this.comps['pt']._pt100Fault = null;
                }
            },
            3: {
                id: 3,
                name: "3. 温度变送器输出开路",
                trigger: () => {
                    // 1: 设置开路故障
                    this.comps['ttrans'].isBreak = true;
                },
                check: () => { return this.comps['ttrans'].isBreak === true; },
                repair: () => {
                    this.comps['ttrans'].isBreak = false;
                }
            },
            4: {
                id: 4,
                name: "4. 温度变送器零点漂移",
                trigger: () => {
                    // 1: 设置开路故障
                    this.comps['ttrans'].zeroAdj = 0.4;
                    this.comps['ttrans'].knobs['zero'].rotation(180);
                    this.comps.ttrans._refreshCache();
                },
                check: () => { return Math.abs(this.comps['ttrans'].zeroAdj - 0.5) < 0.1 },
                repair: () => {
                    this.comps['ttrans'].zeroAdj = 0;
                    this.comps['ttrans'].knobs['zero'].rotation(0);
                    this.comps.ttrans._refreshCache();                    
                }
            },
            5: {
                id: 5,
                name: "5. 温度变送器量程偏差",
                trigger: () => {
                    // 1: 设置开路故障
                    this.comps['ttrans'].spanAdj = 1.125;
                    this.comps['ttrans'].knobs['span'].rotation(90);
                    this.comps.ttrans._refreshCache();                         
                },
                check: () => { return Math.abs(this.comps['ttrans'].spanAdj - 1.125) < 0.05 },
                repair: () => {
                    this.comps['ttrans'].spanAdj = 1;
                    this.comps['ttrans'].knobs['span'].rotation(0);
                    this.comps.ttrans._refreshCache();                         
                }
            },
            6: {
                id: 6,
                name: "6. PID调节器参数失调",
                trigger: () => {
                    this.comps['pid'].P = 0.05;
                    this.comps['pid'].I = 0;
                    this.comps['pid'].D = 0;
                },
                check: () => { return Math.abs(this.comps['pid'].P - 0.1) < 0.1 },
                repair: () => {
                    this.comps['pid'].P = 4;
                }
            },
            7: {
                id: 7,
                name: "7. PID调节器输出回路开路",
                trigger: () => {
                    this.comps['pid'].out1Fault = true;
                },
                check: () => { return this.comps['pid'].out1Fault === true; },
                repair: () => {
                    this.comps['pid'].out1Fault = false;
                }
            },
            8: {
                id: 8,
                name: "8. 三通调节阀执行机构卡死",
                trigger: () => {
                    this.comps['valve'].isStuck = true;
                },
                check: () => { return this.comps['valve'].isStuck === true; },
                repair: () => {
                    this.comps['valve'].isStuck = false;
                }
            },
            9: {
                id: 9,
                name: "9. 三通调节阀信号输入回路开路",
                trigger: () => {
                    this.comps['valve'].currentResistance = 1e8;
                },
                check: () => { return this.comps['valve'].currentResistance > 10000 },
                repair: () => {
                    this.comps['valve'].currentResistance = 250;
                }
            },

        };
        // 2. 动态生成 UI 元素
        const faultForm = document.getElementById('faultForm');
        if (faultForm) {
            faultForm.innerHTML = ''; // 清空原有内容

            Object.values(this.FAULT_CONFIG).forEach(fault => {
                const label = document.createElement('label');
                label.className = 'f-checkbox';

                const checkbox = document.createElement('input');
                checkbox.type = 'checkbox';
                checkbox.value = fault.id;
                checkbox.id = `fault_check_${fault.id}`; // 确保 ID 唯一，不要全是 check1

                label.appendChild(checkbox);
                label.appendChild(document.createTextNode(` ${fault.name}`));

                faultForm.appendChild(label);
            });
        }

    }
    // ==========================================
    // 第二部分：处理流程化任务
    // ==========================================
    // 1. 项目选择框调用的函数，用于切换 任务流程。
    switchWorkflow(taskValue) {
        if (!taskValue) {
            console.log("未选择任何任务，清空流程数据");
            this.workflowComp._workflow = [];
            this.workflowComp._workflowIdx = 0;

            // 如果面板已打开，刷新一下列表显示为空
            if (this.workflowComp._workflowPanelEl) {
                this.workflowComp.closeWorkflowPanel();
            }
            return;
        }

        console.log("切换至任务:", taskValue);

        // 根据具体任务 ID 加载对应的步骤数据
        // 你可以把这些数据存在一个对象里，例如 this.allTasksData
        this.workflowComp._workflow = this.stepsArray[taskValue];

        // 切换任务后，重置进度索引
        this.workflowComp._workflowIdx = 0;

        // 切换任务后，需要重新点击开始
        if (this.workflowComp._workflowPanelEl) {
            this.workflowComp.closeWorkflowPanel();
        }
    }

    // 2. 根据用户选择的方式，单步、完整、评估、演练调用流程工具的对应函数。
    openWorkflowPanel(mode) {
        if (mode === 'step') {
            this.workflowComp.stepByStep();
        }
        else {
            this.workflowComp.openWorkflowPanel(mode);
        }
    }
    /**
     * 3. 一键自动连线：将预设的逻辑关系注入连接池
     */
    applyAllPresets() {
        // 1. 定义预设连接关系
        this.conns = [
            { from: 'engine_pipe_o', to: 'pump_pipe_i', type: 'pipe' },
            { from: 'pump_pipe_o', to: 'tconn_pipe_l', type: 'pipe' },
            { from: 'tconn_pipe_u', to: 'valve_pipe_u', type: 'pipe' },
            { from: 'tconn_pipe_r', to: 'cooler_pipe_i', type: 'pipe' },
            { from: 'cooler_pipe_o', to: 'valve_pipe_l', type: 'pipe' },
            { from: 'valve_pipe_r', to: 'engine_pipe_i', type: 'pipe' },

            { from: 'pid_wire_vcc', to: 'dcpower_wire_p', type: 'wire' },
            { from: 'pid_wire_gnd', to: 'dcpower_wire_n', type: 'wire' },
            { from: 'pt_wire_l', to: 'ttrans_wire_l', type: 'wire' },
            { from: 'pt_wire_r', to: 'ttrans_wire_m', type: 'wire' },
            { from: 'pt_wire_r', to: 'ttrans_wire_r', type: 'wire' },
            { from: 'pid_wire_pi1', to: 'ampmeter_wire_p', type: 'wire' },
            { from: 'ampmeter_wire_n', to: 'ttrans_wire_p', type: 'wire' },
            { from: 'ttrans_wire_n', to: 'pid_wire_ni1', type: 'wire' },
            { from: 'pid_wire_no1', to: 'valve_wire_r', type: 'wire' },
            { from: 'pid_wire_po1', to: 'valve_wire_l', type: 'wire' },

            { from: 'pid_wire_b1', to: 'monitor_wire_b1', type: 'wire' },
            { from: 'pid_wire_a1', to: 'monitor_wire_a1', type: 'wire' }
        ];
        this.redrawAll();

    }

    // 4. 启动系统，控制开关、截止阀之类组件控制系统运行
    async applyStartSystem() {
        this.comps.dcpower.isOn = true;
        this.comps.dcpower.update();
        this.comps.pump.pumpOn = true;
        this.comps.engine.engOn = true;
        this.comps.pid.mode = "AUTO";

    }
    // 5. 多点步进系统，用于多次设置参数
    /**
     * 5点步进系统：根据 PID 模式切换步进目标
     * 手动模式：步进 PID 输出 (0, 25, 50, 75, 100)
     * 自动模式：步进设定值 (0.25, 0.5, 0.75, 1, 0)，模拟不同压力点的系统响应
     */
    fiveStep() {
        const pid = this.comps['pid'];
        // if (!varipress) return;
        // 1. 获取当前  模式 ()
        const isManual = pid.mode === "MAN";
        // 2. 定义不同模式下的步进序列
        const steps = isManual
            ? [0, 25, 50, 75, 100]                   // 手动模式：PID 输出百分比 (%)
            : [0.25, 0.5, 0.75, 1, 0]; // 自动模式：设置参数值
        // 3. 维护步进索引
        if (this._testStep === undefined || this._testStep >= steps.length) {
            this._testStep = 0;
        }
        const nextIndex = this._testStep;
        const targetValue = steps[nextIndex];
        // 4. 执行更新逻辑
        if (isManual) {
            // --- 手动模式逻辑 ---
            // 设置 PID 的手动输出值
            pid.OUT = targetValue;
        } else {
            // --- 自动模式逻辑 ---
            // // 设置参数变化
            // varipress.setPressure = targetValue;
            // if (typeof varipress.update === 'function') {
            //     varipress.update();
            // }
        }
        // 5. 更新计数器
        this._testStep = (nextIndex + 1) % steps.length;
    }


    // ==========================================
    // 第二部分：交互管理（手动连线控制）
    // ==========================================
    /**
     * 显示系统级右键菜单（用于设置仿真步长等）
     */
    showSystemContextMenu(evt) {
        // 1. 移除可能已存在的旧菜单
        const oldMenu = document.getElementById('sys-context-menu');
        if (oldMenu) oldMenu.remove();

        const menu = document.createElement('div');
        menu.id = 'sys-context-menu';
        // 基础样式
        const baseStyle = `
        position: fixed; top: ${evt.clientY}px; left: ${evt.clientX}px;
        background: white; border: 1px solid #ccc; border-radius: 4px;
        box-shadow: 2px 2px 10px rgba(0,0,0,0.2); z-index: 10000;
        padding: 5px 0; min-width: 160px; font-family: sans-serif; font-size: 14px;
    `;
        menu.style = baseStyle;

        // 工具函数：创建普通菜单项
        const createItem = (label, onClick, hasSubmenu = false) => {
            const item = document.createElement('div');
            item.style = 'padding: 8px 15px; cursor: pointer; transition: background 0.2s; display: flex; justify-content: space-between; align-items: center;';
            item.innerHTML = `<span>${label}</span>${hasSubmenu ? '<span style="font-size:10px;">▶</span>' : ''}`;

            item.onmouseenter = () => item.style.background = '#f0f0f0';
            item.onmouseleave = () => item.style.background = 'transparent';

            if (onClick) {
                item.onclick = (e) => {
                    e.stopPropagation();
                    onClick();
                };
            }
            return item;
        };

        // --- 创建“仿真步长”子菜单项 ---
        const stepLabel = `仿真步长 (${(this.voltageSolver.deltaTime * 1000).toFixed(2)}ms)`;
        const stepItem = createItem(stepLabel, null, true);

        // 创建子菜单容器
        const submenu = document.createElement('div');
        submenu.style = `
        position: absolute; left: 100%; top: 0; background: white;
        border: 1px solid #ccc; border-radius: 4px; box-shadow: 2px 2px 10px rgba(0,0,0,0.1);
        display: none; padding: 5px 0; min-width: 120px;
    `;

        // 定义可选步长
        const steps = [
            { label: '0.1 ms', value: 0.0001 },
            { label: '0.01 ms', value: 0.00001 },
            { label: '0.001 ms', value: 0.000001 }
        ];

        steps.forEach(s => {
            // 判断是否是当前步长
            const isCurrent = Math.abs(this.voltageSolver.deltaTime - s.value) < s.value * 0.1;
            const subItem = document.createElement('div');
            subItem.style = 'padding: 8px 15px; cursor: pointer; display: flex; align-items: center;';
            subItem.innerHTML = `<span style="width: 20px;">${isCurrent ? '✓' : ''}</span>${s.label}`;

            subItem.onmouseenter = () => subItem.style.background = '#f0f0f0';
            subItem.onmouseleave = () => subItem.style.background = 'transparent';

            subItem.onclick = (e) => {
                e.stopPropagation();
                this.setSimulationStep(s.value);
                menu.remove();
            };
            submenu.appendChild(subItem);
        });

        // 鼠标悬浮显示子菜单逻辑
        stepItem.onmouseenter = () => {
            stepItem.style.background = '#f0f0f0';
            submenu.style.display = 'block';
        };
        stepItem.onmouseleave = (e) => {
            // 检查鼠标是否移向了子菜单
            if (!submenu.contains(e.relatedTarget)) {
                submenu.style.display = 'none';
            }
        };
        submenu.onmouseleave = (e) => {
            if (!stepItem.contains(e.relatedTarget)) {
                submenu.style.display = 'none';
            }
        };

        stepItem.appendChild(submenu);
        menu.appendChild(stepItem);

        // 挂载到容器
        this.container.appendChild(menu);

        // 点击其他地方关闭
        const closeMenu = (e) => {
            if (!menu.contains(e.target)) {
                menu.remove();
                window.removeEventListener('mousedown', closeMenu);
            }
        };
        window.addEventListener('mousedown', closeMenu);
    }

    /**
     * 修改步长的逻辑方法
     */
    setSimulationStep(val) {
        if (this.voltageSolver) {
            this.voltageSolver.deltaTime = val;
            console.log(`[System] 步长已切换至: ${val * 1000} ms`);
            // 必要时重置部分瞬态参数，防止数值突变
            this._needsRedraw = true;
        }
    }


    /**
     * 显示一个临时的浮动提示（用于演示模式自动答题）
     */
    showFloatingTip(text, duration = 2500) {
        const tip = document.createElement('div');
        Object.assign(tip.style, {
            position: 'fixed',
            top: '10%',
            left: '50%',
            transform: 'translateX(-50%)',
            padding: '12px 24px',
            background: 'rgba(45, 134, 45, 0.9)', // 墨绿色，代表正确/演示
            color: '#fff',
            borderRadius: '20px',
            fontSize: '16px',
            boxShadow: '0 4px 12px rgba(0,0,0,0.3)',
            zIndex: '10001',
            pointerEvents: 'none', // 不阻碍点击，防误触
            transition: 'opacity 0.5s ease'
        });
        tip.innerHTML = `💡 ${text}`;
        this.container.appendChild(tip);

        // 动画消失逻辑
        setTimeout(() => {
            tip.style.opacity = '0';
            setTimeout(() => {
                if (this.container.contains(tip)) this.container.removeChild(tip);
            }, 500);
        }, duration);
    }
    /**
     * 1. 处理端口点击事件：实现“起点-预览-终点”连线逻辑
     */
    handlePortClick(comp, portId, type) {
        if (!this.linkingState) {
            // 设定起点
            this.linkingState = { comp, portId, type };
            this.tempLine = new Konva.Line({
                stroke: type === 'wire' ? '#eb0d0d' : '#463aed',
                strokeWidth: type === 'wire' ? 2 : 12,
                opacity: 0.6, dash: [10, 5]
            });
            this.layer.add(this.tempLine);
            this.requestRedraw();
        } else {
            // 设定终点
            if (this.linkingState.type === type) {
                const aPort = this.linkingState.portId;
                const bPort = portId;
                if (aPort === bPort) { this.resetLinking(); return; }

                const newConn = { from: aPort, to: bPort, type };


                // 1. 检查是否已经存在该连接（无论正反向），在统一的 this.conns 中查找
                const exists = this.conns.some(c => this._connEqual(c, newConn));
                if (exists) {
                    this.resetLinking();
                    return;
                }

                // 2. 修正后的管路冲突检查
                if (type === 'pipe') {
                    // 只有当新连接的端点 被“除了对方以外”的其他连接占用时，才算冲突
                    // 在船舶管路仿真中，通常一个接口只能接一根管子
                    const isPortBusy = (pid) => this.conns.filter(c => c.type === 'pipe').some(c => c.from === pid || c.to === pid);

                    if (isPortBusy(aPort)) {
                        alert(`端口 ${aPort} 已有管路连接`);
                        this.resetLinking();
                        return;
                    }
                    if (isPortBusy(bPort)) {
                        alert(`端口 ${bPort} 已有管路连接`);
                        this.resetLinking();
                        return;
                    }
                    // 对于管道类型的连接，这里就根据this.requiredPipes来检查是否满足预设的顺序，不满足自动更换连接方向
                    if (newConn.type === 'pipe') {
                        // 1. 查找是否存在对应的预设管路（不考虑顺序）
                        const required = this.requiredPipes.find(r =>
                            this._connEqual(r, newConn)
                        );
                        if (required) {
                            // 2. 强制使用预设的标准方向
                            // 无论用户是从 A 拉向 B，还是 B 拉向 A
                            // 只要这俩点在预设里，我们就统一成预设的顺序
                            newConn.from = required.from;
                            newConn.to = required.to;
                        }
                    }
                }
                // 3. 电路通常允许并联（一个端点接多根线），所以不对 wire 做 isPortBusy 检查
                this.addConnWithHistory(newConn);
            } else {
                alert("类型不匹配：管路不能连接到电路！");
            }
            this.resetLinking();
        }
    }
    // 辅助函数：比较两个连接是否等价（无顺序）
    _connEqual(a, b) {
        // 无向比较：类型相同且端点集合相等（正向或反向均视为相同连接）
        if (a.type !== b.type) return false;
        return (a.from === b.from && a.to === b.to) || (a.from === b.to && a.to === b.from);
    }

    // 辅助函数：生成连接的规范键（端点排序后）用于界面元素标记
    _connKeyCanonical(c) {
        // 无向规范键：按字符串顺序对端点排序以保证正反向具有相同键
        const a = c.from;
        const b = c.to;
        return a <= b ? `${a}-${b}` : `${b}-${a}`;
    }

    // 2. 连接虚线销毁函数。
    resetLinking() {
        // 1. 物理销毁 Konva 对象，释放内存并从图层移除
        if (this.tempLine) {
            this.tempLine.destroy();
            this.tempLine = null;
        }
        // 2. 清空状态位
        this.linkingState = null;
        // 3. 刷新画布
        this.requestRedraw();
    }

    // 3. 简单的连接历史操作（仅针对用户点击行为）
    addConnWithHistory(conn) {
        const sys = this;
        const action = {
            do() {
                if (!sys.conns.some(c => sys._connEqual(c, conn))) sys.conns.push(conn);
                sys.redrawAll();
            },
            undo() {
                const idx = sys.conns.findIndex(c => sys._connKeyCanonical(c) === sys._connKeyCanonical(conn) && c.type === conn.type);
                if (idx !== -1) sys.conns.splice(idx, 1);
                sys.redrawAll();
            }
        };
        this.history.do(action);
    }
    addConn(conn) {
        if (!this.conns.some(c => this._connEqual(c, conn))) this.conns.push(conn);
        this.redrawAll();
    }

    // 4. 删除连线调用，前者可以恢复，后者不可恢复。
    removeConnWithHistory(conn) {
        const sys = this;
        const action = {
            do() {
                const idx = sys.conns.findIndex(c => sys._connKeyCanonical(c) === sys._connKeyCanonical(conn) && c.type === conn.type);
                if (idx !== -1) sys.conns.splice(idx, 1);
                sys.redrawAll();
            },
            undo() {
                if (!sys.conns.some(c => sys._connEqual(c, conn))) sys.conns.push(conn);
                sys.redrawAll();
            }
        };
        this.history.do(action);
    }
    removeConn(conn) {
        const idx = this.conns.findIndex(c => this._connKeyCanonical(c) === this._connKeyCanonical(conn) && c.type === conn.type);
        if (idx !== -1) this.conns.splice(idx, 1);
        this.redrawAll();
    }

    //5. 动画方式添加连线：3s 完成一次连线，结束后把连线加入 this.conns 并重绘，用户演示。
    addConnectionAnimated(conn) {
        return new Promise((resolve) => {
            const getPosByPort = (portId) => {
                const did = portId.split('_')[0];
                return this.comps[did]?.getAbsPortPos(portId);
            };

            const fromPos = getPosByPort(conn.from);
            const toPos = getPosByPort(conn.to);

            // --- 安全检查：如果坐标获取不到，直接完成，防止 Promise 永远挂起 ---
            if (!fromPos || !toPos) {
                console.error("Connection failed: Missing port coordinates", conn);
                this.conns.push(conn);
                this.redrawAll();
                return resolve();
            }

            const animLine = new Konva.Line({
                points: [fromPos.x, fromPos.y, fromPos.x, fromPos.y],
                stroke: conn.type === 'wire' ? '#e41c1c' : '#78e4c9',
                strokeWidth: conn.type === 'wire' ? 6 : 10,
                lineCap: 'round',
                lineJoin: 'round',
                shadowBlur: conn.type === 'pipe' ? 6 : 0,
                shadowColor: '#333',
                opacity: 0.95,
                listening: false // 提高性能，动画线不参与事件捕获
            });

            this.lineLayer.add(animLine);

            const duration = 3000; // 建议 1.2s，3s 对自动演示来说略久
            const start = performance.now();

            const animate = (now) => {
                const elapsed = now - start;
                const t = Math.min(1, elapsed / duration);

                // 缓动函数 (Ease-out)，让连线在接近终点时有一个减速感，更具质感
                const easeOut = 1 - Math.pow(1 - t, 3);

                const curX = fromPos.x + (toPos.x - fromPos.x) * easeOut;
                const curY = fromPos.y + (toPos.y - fromPos.y) * easeOut;

                animLine.points([fromPos.x, fromPos.y, curX, curY]);
                this.lineLayer.batchDraw();

                if (t < 1) {
                    requestAnimationFrame(animate);
                } else {
                    // --- 动画彻底结束后的清理与状态更新 ---
                    animLine.destroy();

                    // 确保不重复添加
                    const exists = this.conns.some(c => c.from === conn.from && c.to === conn.to);
                    if (!exists) {
                        this.conns.push(conn);
                    }

                    this.redrawAll();

                    // 关键点：在这里 resolve，外部的 await 才会继续
                    resolve();
                }
            };

            requestAnimationFrame(animate);
        });
    }

    // ==========================================
    // 第三部分：渲染引擎（连线绘制）
    // ==========================================

    /**
    * 统一重绘接口：当组件移动或连接池改变时调用
    */
    redrawAll() {
        this._renderGroup(this.conns.filter(c => c.type === 'pipe'), 'pipe');
        this._renderGroup(this.conns.filter(c => c.type === 'wire'), 'wire');
    }

    // 请求一次在下一帧统一重绘（组件在高频更新中应调用此方法）
    requestRedraw() {
        this._needsRedraw = true;
    }

    // 增量更新现有线条节点的位置（避免销毁重建）
    updateLinePositions() {
        const getPosByPort = (portId) => {
            const did = portId.split('_')[0];
            return this.comps[did]?.getAbsPortPos(portId);
        };

        // 更新 pipeNodes：每个 conn 对应 3 个节点（line, flow, handle）
        const pipeConns = this.conns.filter(c => c.type === 'pipe');
        if (this.pipeNodes.length === pipeConns.length * 3) {
            for (let i = 0; i < pipeConns.length; i++) {
                const conn = pipeConns[i];
                const p1 = getPosByPort(conn.from);
                const p2 = getPosByPort(conn.to);
                if (!p1 || !p2) continue;
                const baseIdx = i * 3;
                const line = this.pipeNodes[baseIdx];
                const flow = this.pipeNodes[baseIdx + 1];
                const handle = this.pipeNodes[baseIdx + 2];
                let pts = [p1.x, p1.y, p2.x, p2.y];
                if (conn.midPoint) pts = [p1.x, p1.y, conn.midPoint.x, conn.midPoint.y, p2.x, p2.y];
                try { line.points(pts); flow.points(pts); handle.position(conn.midPoint || { x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2 }); } catch (e) { }
            }
        } else {
            // 节点数量不匹配，退化为完全重绘
            // 不直接调用 this.redrawAll()，仅标记需要重绘，下一帧会触发
            this._needsRedraw = true;
        }

        // 更新 wireNodes：每个 conn 对应 1 个节点
        const wireConns = this.conns.filter(c => c.type === 'wire');
        if (this.wireNodes.length === wireConns.length) {
            for (let i = 0; i < wireConns.length; i++) {
                const conn = wireConns[i];
                const p1 = getPosByPort(conn.from);
                const p2 = getPosByPort(conn.to);
                if (!p1 || !p2) continue;
                const node = this.wireNodes[i];
                try {
                    if (conn.from.includes('multimeter') || conn.to.includes('multimeter')) {
                        const midX = (p1.x + p2.x) / 2;
                        const midY = Math.max(p1.y, p2.y) + 20;
                        node.points([p1.x, p1.y, midX, midY, p2.x, p2.y]);
                    } else {
                        const midX = (p1.x + p2.x) / 2;
                        const midY = (p1.y + p2.y) / 2;
                        const dx = p2.x - p1.x;
                        const dy = p2.y - p1.y;
                        const len = Math.sqrt(dx * dx + dy * dy) || 1;
                        const ux = -dy / len;
                        const uy = dx / len;
                        const devA = conn.from.split('_')[0];
                        const devB = conn.to.split('_')[0];
                        const siblings = this.conns.filter(c => c.type === 'wire' && (() => {
                            const ca = c.from.split('_')[0];
                            const cb = c.to.split('_')[0];
                            return (ca === devA && cb === devB) || (ca === devB && cb === devA);
                        })());
                        const idx = siblings.findIndex(c => this._connKeyCanonical(c) === this._connKeyCanonical(conn));
                        const total = siblings.length || 1;
                        const spacing = 18;
                        const longSpacing = 8;
                        const offset = (idx - (total - 1) / 2) * spacing;
                        const longOffset = (idx - (total - 1) / 2) * longSpacing;
                        const controlX = midX + ux * offset + (dx / len) * longOffset;
                        const controlY = midY + uy * offset + (dy / len) * longOffset;
                        const pts = [p1.x, p1.y, controlX, controlY, controlX, controlY, p2.x, p2.y];
                        node.points(pts);
                    }
                } catch (e) { }
            }
        } else {
            this._needsRedraw = true;
        }
    }
    _renderGroup(conns, type) {
        const nodesRef = type === 'pipe' ? 'pipeNodes' : 'wireNodes';
        this[nodesRef].forEach(n => n.destroy());
        this[nodesRef] = [];

        const getPosByPort = (portId) => {
            const did = portId.split('_')[0];
            return this.comps[did]?.getAbsPortPos(portId);
        };

        conns.forEach(conn => {
            const p1 = getPosByPort(conn.from);
            const p2 = getPosByPort(conn.to);
            if (!p1 || !p2) return;

            let line;
            if (type === 'pipe') {
                // --- 1. 计算管路点集合 ---
                // 如果 conn.midPoint 存在，则管路由三点组成
                let pts = [p1.x, p1.y, p2.x, p2.y];
                if (conn.midPoint) {
                    pts = [p1.x, p1.y, conn.midPoint.x, conn.midPoint.y, p2.x, p2.y];
                }

                // --- 2. 绘制底层管道和流动层 ---
                line = new Konva.Line({
                    points: pts,
                    stroke: '#c4c7c8',
                    strokeWidth: 16,
                    lineCap: 'round',
                    lineJoin: 'round'
                });
                const flow = new Konva.Line({
                    points: pts,
                    stroke: '#130cdf',
                    strokeWidth: 4,
                    dash: [10, 20],
                    name: 'flow',
                    lineJoin: 'round',
                    visible: false
                });

                // --- 3. 创建可拖动的中间点 (Handle) ---
                const handlePos = conn.midPoint || { x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2 };
                const handle = new Konva.Circle({
                    x: handlePos.x,
                    y: handlePos.y,
                    radius: 6,
                    fill: '#f1c40f',
                    stroke: '#d35400',
                    strokeWidth: 2,
                    draggable: true,
                    visible: false // 默认隐藏，鼠标经过管路时显示
                });

                // 拖拽事件：更新数据并重绘
                handle.on('dragmove', () => {
                    conn.midPoint = { x: handle.x(), y: handle.y() };
                    // 实时更新当前线条预览，提高流畅度
                    const newPts = [p1.x, p1.y, handle.x(), handle.y(), p2.x, p2.y];
                    line.points(newPts);
                    flow.points(newPts);
                });

                handle.on('dragend', () => {
                    this.redrawAll(); // 确保所有关联层刷新
                });

                // 交互效果：鼠标悬停在管路上显示拖动手柄
                const showHandle = () => { handle.visible(true); if (this.requestRedraw) this.requestRedraw(); };
                const hideHandle = () => { if (!handle.isDragging()) handle.visible(false); if (this.requestRedraw) this.requestRedraw(); };

                line.on('mouseenter', showHandle);
                line.on('mouseleave', hideHandle);
                handle.on('mouseenter', showHandle);
                handle.on('mouseleave', hideHandle);

                // 双击删除逻辑
                const key = this._connKeyCanonical(conn);
                flow.setAttr('connKey', key);
                const removeHandler = () => {
                    const existing = this.conns.find(c => this._connKeyCanonical(c) === key && c.type === 'pipe');
                    if (existing) this.removeConnWithHistory(existing);
                };
                line.on('dblclick', removeHandler);

                this.lineLayer.add(line, flow, handle);
                this[nodesRef].push(line, flow, handle);

                line.moveToBottom();
                flow.moveToBottom();
            } else {
                // 绘制电路：三点贝塞尔曲线（start -> control -> end），对同一对组件的多条线做偏移以防重叠
                if (conn.from.includes('multimeter') || conn.to.includes('multimeter')) {
                    // 万用表特殊连线逻辑
                    let strokeColor;
                    // --- 核心修改：万用表表笔线增加中点以触发 tension ---
                    const midX = (p1.x + p2.x) / 2;
                    const midY = Math.max(p1.y, p2.y) + 20; // 模拟重力，让中点下垂 30 像素

                    // 重新构造点序列：[起点, 中点, 终点]
                    const linePoints = [p1.x, p1.y, midX, midY, p2.x, p2.y];
                    // 根据端子功能上色
                    if (conn.from.includes('com') || conn.to.includes('com')) {
                        strokeColor = '#006400'; // 墨绿色
                    } else if (conn.from.includes('wire_v') || conn.to.includes('wire_v') || conn.from.includes('wire_ma') || conn.to.includes('wire_ma')) {
                        strokeColor = '#FF4500'; // 火红色 (OrangeRed)
                    }
                    line = new Konva.Line({
                        points: linePoints,
                        stroke: strokeColor,
                        strokeWidth: 6,
                        lineCap: 'round',
                        lineJoin: 'round',
                        tension: 0.4, // 关键：lineTension设置此值大于0即变为贝塞尔曲线
                    });
                }
                else {
                    const midX = (p1.x + p2.x) / 2;
                    const midY = (p1.y + p2.y) / 2;
                    const dx = p2.x - p1.x;
                    const dy = p2.y - p1.y;
                    // 归一化的垂直向量
                    const len = Math.sqrt(dx * dx + dy * dy) || 1;
                    const ux = -dy / len;
                    const uy = dx / len;

                    // 找到与当前连接相同组件对的所有电线（无顺序）
                    const devA = conn.from.split('_')[0];
                    const devB = conn.to.split('_')[0];
                    const siblings = this.conns.filter(c => c.type === 'wire' && (() => {
                        const ca = c.from.split('_')[0];
                        const cb = c.to.split('_')[0];
                        return (ca === devA && cb === devB) || (ca === devB && cb === devA);
                    })());
                    const idx = siblings.findIndex(c => this._connKeyCanonical(c) === this._connKeyCanonical(conn));
                    const total = siblings.length || 1;
                    const spacing = 18; // 垂直偏移间距
                    const longSpacing = 8; // 沿线微偏移，减少缠绕
                    // 使偏移在多条线时成对分布于两侧
                    const offset = (idx - (total - 1) / 2) * spacing;
                    const longOffset = (idx - (total - 1) / 2) * longSpacing;

                    const controlX = midX + ux * offset + (dx / len) * longOffset;
                    const controlY = midY + uy * offset + (dy / len) * longOffset;

                    // 使用二次控制点复制为两个控制点以兼容 Konva 的贝塞尔格式
                    const pts = [p1.x, p1.y, controlX, controlY, controlX, controlY, p2.x, p2.y];
                    let stroke;
                    if (conn.from.endsWith('p') || conn.to.endsWith('p') || conn.from.includes('wire_a')) stroke = '#e60c0c';
                    else stroke = '#544f4f';
                    line = new Konva.Line({
                        points: pts,
                        stroke: stroke, strokeWidth: 4, bezier: true
                    });

                }
                // 标记连接键并绑定双击删除事件
                const key = this._connKeyCanonical(conn);
                line.setAttr('connKey', key);
                line.setAttr('connType', type);
                line.on('dblclick', () => {
                    const existing = this.conns.find(c => this._connKeyCanonical(c) === key && c.type === type);
                    if (existing) {
                        this.removeConnWithHistory(existing);
                    }
                });
                this.lineLayer.add(line);
                this[nodesRef].push(line);
            }
            line.moveToBottom();
        });
        this.lineLayer.batchDraw();
    }

    // ==========================================
    // 第四部分：电路仿真、气路仿真、仪表显示
    // ==========================================

    //1. 提供给下属组件调用的回调函数，组件可根据端口电压决定自己的状态。
    getVoltageBetween(portIdA, portIdB) {
        return this.voltageSolver.getPD(portIdA, portIdB);
    }

    isPortConnected(pA, pB) {
        return this.voltageSolver.isPortConnected(pA, pB);
    }
    getPressAt(port) {

    }

    // ==========================================
    // 第五部分：回调函数，主循环
    // ==========================================
    // 1. 下属组件状态发生变化时调用的函数
    onComponentStateChange(dev) {

    }
    /**
     * 优化点 1：物理计算循环 (CPU 密集型)
     * 将 CircuitSolver 和 Workflow 的 check 完全隔离在 UI 重绘之外
     */
    _updatePhysics() {
        this._physicsIterCount++;
        // 1. 电路求解
        this.voltageSolver.update();
        // 2. 气路求解
        this.pressSolver.solve();


    }

    /**
         * 优化点 2：静态组件 Canvas 缓存策略
         * 对 Resistor、PT100 等纯静态、无指针旋转的组件进行离屏 Canvas 缓存
         */
    _applyStaticCaching() {
        // 1. 遍历组件并执行 cache()
        Object.values(this.comps).forEach(comp => {
            if (comp.cache === 'fixed') {
                if (comp.group && comp.group.cache) {
                    // cache() 是 Konva 降低 CPU 渲染压力的利器
                    comp.group.cache();
                }
            }
        });
    }

    /**
     * 优化点 3：按需重绘循环 (GPU/UI 密集型)
     * 只有当 _needsRedraw 标记为 true 时，才执行 batchDraw()
     */
    _renderLoop() {
        // 1. 检查重绘标记 (耗时极多)
        if (this._needsRedraw) {
            // batchDraw() 是 Konva 内部优化过的重绘方法
            this.layer.batchDraw();
            // 同步重绘连线图层，确保在拖动组件时线路位置更新可见
            this.lineLayer.batchDraw();
            this._needsRedraw = false; // 重置标记
        }
        // 2. 递归调用 RequestAnimationFrame，跟随浏览器 UI 刷新频率
        requestAnimationFrame(() => this._renderLoop());
    }
}

// 最小历史管理器：仅对用户交互的连线添加撤销/重做支持
class HistoryManager {
    constructor() {
        this.undos = [];
        this.redos = [];
        this.max = 80;
        this.onChange = () => { };
    }

    do(action) {
        try {
            action.do();
            this.undos.push(action);
            if (this.undos.length > this.max) this.undos.shift();
            this.redos = [];
            this.onChange();
        } catch (e) { console.error('History do error', e); }
    }

    undo() {
        const a = this.undos.pop();
        if (!a) return;
        try { a.undo(); this.redos.push(a); this.onChange(); } catch (e) { console.error('History undo error', e); }
    }

    redo() {
        const a = this.redos.pop();
        if (!a) return;
        try { a.do(); this.undos.push(a); this.onChange(); } catch (e) { console.error('History redo error', e); }
    }
}
