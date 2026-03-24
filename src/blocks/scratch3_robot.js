const Cast = require('../util/cast');
const MathUtil = require('../util/math-util');
const Timer = require('../util/timer');
//const Robot_command = require('../robot/robot_command');
const Renderer = require('scratch-render');
const MOTORS_ON_DELTA = 50;
const DEGREE_RATIO = 5.19;
const SIM_SENSOR_PROBES_STORAGE_KEY = 'rs3.simSensorProbes';
const DEFAULT_SIM_SENSOR_PROBES = [
    {localX: 1, localY: 38, direction: 'forward'},
    {localX: 18, localY: 30, direction: 'forward'},
    {localX: 18, localY: -38, direction: 'backward'},
    {localX: -14, localY: -38, direction: 'backward'},
    {localX: -16.5, localY: 30, direction: 'forward'}
];

class Scratch3RobotBlocks {
    constructor (runtime) {
        /**
         * The runtime instantiating this block package.
         * @type {Runtime}
         */
        this.runtime = runtime;
    //    this.robot_command = new Robot_command();

        this.power_in_percent_left = 50;
        this.power_in_percent_right = 50;
      //  this.power = Math.round(this.power_in_percent* 0.63);
        this.power_left =   Math.round(this.power_in_percent_left * 0.63);
        this.power_right =  Math.round(this.power_in_percent_right * 0.63);
        this.motors_on_interval = null;
        this.motors_off_interval = null;
        this.need_to_stop = true; //false
        this.robot_direction = 'direction_forward';
        this.time = Date.now();
        this.motors_on_time_delta = null;

        this.motors_on_time1 = Date.now();
        this.motors_on_time2 = Date.now();

        this.motors_on_loop_need = null;

        this.robot_motors_on_for_seconds_timeout = null;
        this.robot_motors_on_for_seconds_timeout_stop = null;

        this.command_sent = false;

        this.a_command_unblock_interval = null;

        this.set_direction_block_step = 1;

        this.is_motors_on_active = false;

        this.time_sent1 = Date.now();
        this.time_sent2 = Date.now();
        this.time_sent3 = Date.now();

       this.robot_motors_on_for_seconds_end_timeout = null;


        this.runtime.RCA.registerRobotIsScratchduinoCallback(() => {

          this.power_left =  63;
          this.power_right =  63;

        });

        this.runtime.RCA.registerRobotIsRobboCallback(() => {

          this.power_left  =  Math.round(this.power_in_percent_left * 0.63);
          this.power_right =  Math.round(this.power_in_percent_right * 0.63);


        });

        this.runtime.are_motors_inverted = false; 

        this.runtime.left_motor_inverted = false; 
        this.runtime.right_motor_inverted = false; 

        this.runtime.sim_ac=false;
        this.runtime.going=false;
        this.kW=0.01; // шаг движения робота
        this.fps = 6; // Скорость обработки кадров инверсивно меняется 1000/5 = 200   = 72 клетки  =
        this.rad= 883;// угол поворопа меняет
        this.xc= 0;
        this.yc=0;
        this.sim_dist_l=0;
        this.sim_dist_r=0;
        this.wall_color=[0,0,0];
        this.sim_pl = 63;
        this.sim_pr = 63;
        this.sim_int = null;
        this.distl=0;
        this.distr=0;
        this.minidist = 0;
        this.robot_delta_x=0;
        this.robot_delta_y=0;
        this.ddx =0;
        this.ddy = 0;
        this.ddp=[];  this.ddp[0]=0; this.ddp[1]=0; this.ddp[2]=0;
        this.ddd = []; this.ddd[0]=0;
        this.ddc = [];this.ddc[0]=1;this.ddc[1]=1;this.ddc[2]=1;this.ddc[3]=1;
        this.ddl = 0 ;
        this.last_util = {};
        this.start_deg=0;
        this.runtime.on('PROJECT_STOP_ALL', this._onProjectStopAll.bind(this));
        this.simSensorProbes = this._loadSimSensorProbes();
    }

  _getDefaultSimSensorProbes () {
    return DEFAULT_SIM_SENSOR_PROBES.map(p => ({
      localX: p.localX,
      localY: p.localY,
      direction: p.direction
    }));
  }

  _sanitizeSimSensorProbe (probe, fallbackProbe) {
    const localX = Number(probe && probe.localX);
    const localY = Number(probe && probe.localY);
    const direction = (probe && probe.direction === 'backward') ? 'backward' : 'forward';
    return {
      localX: Number.isFinite(localX) ? localX : fallbackProbe.localX,
      localY: Number.isFinite(localY) ? localY : fallbackProbe.localY,
      direction: direction
    };
  }

  _loadSimSensorProbes () {
    const defaults = this._getDefaultSimSensorProbes();
    try {
      const storage = (typeof globalThis !== 'undefined') ? globalThis.localStorage : null;
      if (!storage) return defaults;
      const raw = storage.getItem(SIM_SENSOR_PROBES_STORAGE_KEY);
      if (!raw) return defaults;
      const parsed = JSON.parse(raw);
      if (!Array.isArray(parsed)) return defaults;
      return defaults.map((def, idx) => this._sanitizeSimSensorProbe(parsed[idx], def));
    } catch (e) {
      return defaults;
    }
  }

  _saveSimSensorProbes () {
    try {
      const storage = (typeof globalThis !== 'undefined') ? globalThis.localStorage : null;
      if (!storage) return;
      storage.setItem(SIM_SENSOR_PROBES_STORAGE_KEY, JSON.stringify(this.simSensorProbes));
    } catch (e) {
      // Ignore storage write errors in restricted environments.
    }
  }

  setSimSensorProbeConfig (index, localX, localY, direction) {
    const probeIndex = Number(index) - 1;
    if (probeIndex < 0 || probeIndex >= this.simSensorProbes.length) return false;
    const fallback = this.simSensorProbes[probeIndex];
    this.simSensorProbes[probeIndex] = this._sanitizeSimSensorProbe({localX, localY, direction}, fallback);
    this._saveSimSensorProbes();
    return true;
  }

  getSimSensorProbeConfig () {
    return this.simSensorProbes.map((probe, idx) => ({
      sensorIndex: idx + 1,
      localX: probe.localX,
      localY: probe.localY,
      direction: probe.direction
    }));
  }

  resetSimSensorProbeConfig () {
    this.simSensorProbes = this._getDefaultSimSensorProbes();
    this._saveSimSensorProbes();
    return this.getSimSensorProbeConfig();
  }

    /**
     * Stop robot motors and clear timers when project is stopped (Stop button).
     */
    _onProjectStopAll () {
        if (this.robot_motors_on_for_seconds_timeout_stop != null) {
            clearTimeout(this.robot_motors_on_for_seconds_timeout_stop);
            this.robot_motors_on_for_seconds_timeout_stop = null;
        }
        if (this.robot_motors_on_for_seconds_end_timeout != null) {
            clearTimeout(this.robot_motors_on_for_seconds_end_timeout);
            this.robot_motors_on_for_seconds_end_timeout = null;
        }
        if (this.motors_on_interval != null) {
            clearInterval(this.motors_on_interval);
            this.motors_on_interval = null;
        }
        if (this.motors_off_interval != null) {
            clearInterval(this.motors_off_interval);
            this.motors_off_interval = null;
        }
        if (this.sim_int != null) {
            clearInterval(this.sim_int);
            this.sim_int = null;
        }
        if (this.a_command_unblock_interval != null) {
            clearInterval(this.a_command_unblock_interval);
            this.a_command_unblock_interval = null;
        }
        this.runtime.going = false;
        this.need_to_stop = true;
        this.is_motors_on_active = false;
        if (this.runtime.RCA && !this.runtime.sim_ac) {
            this.runtime.RCA.setRobotPower(0, 0, 0);
            this.runtime.RCA.unblockPowerCommand();
            this.runtime.RCA.unblock_A_CommandQueue();
        }
    }

    /**
     * Retrieve the block primitives implemented by this package.
     * @return {object.<string, Function>} Mapping of opcode to Function.
     */
    getPrimitives () {
        return {
            robot_motors_on_for_seconds: this.robot_motors_on_for_seconds,
            robot_motors_on: this.robot_motors_on,
            robot_motors_off:this.robot_motors_off,
            robot_set_direction_to:this.robot_set_direction_to,
            robot_set_motors_left_right_power_and_direction_separately:this.robot_set_motors_left_right_power_and_direction_separately,
            robot_get_sensor_data: this.robot_get_sensor_data,
            robot_motors_on_for_steps:this.robot_motors_on_for_steps,
            robot_turnright: this.robot_turnright,
            robot_turnleft: this.robot_turnleft,
            robot_set_motors_power:this.robot_set_motors_power,
            robot_set_motors_power_left_right_separately:this.robot_set_motors_power_left_right_separately,
            robot_start_button_pressed:this.robot_start_button_pressed,
            robot_turn_led_on:this.robot_turn_led_on,
            robot_turn_led_off:this.robot_turn_led_off,
            robot_claw_closed:this.robot_claw_closed,
            robot_claw_state:this.robot_claw_state,
            robot_reset_trip_meters: this.robot_reset_trip_meters,
            robot_get_rgb_sensor_data: this.robot_get_rgb_sensor_data,
            robot_is_current_color:this.robot_is_current_color,
            robot_set_sens:this.robot_set_sens,
            robot_get_dist:this.robot_get_dist,
            robot_touch:this.robot_touch,
            robot_wall_color:this.robot_wall_color,
            getSensorDataFromLastUtil:this.getSensorDataFromLastUtil,
            getSimSensorDebugData:this.getSimSensorDebugData,
            setSimSensorProbeConfig:this.setSimSensorProbeConfig,
            getSimSensorProbeConfig:this.getSimSensorProbeConfig,
            resetSimSensorProbeConfig:this.resetSimSensorProbeConfig,
            robot_first_draw:this.robot_first_draw
        };
    }

  getMonitored () {
        return {

        };
    }

  getSensorDataFromLastUtil(index){
  //  console.warn(this.runtime.util);
//    console.warn(this.last_util);

//console.warn(this.last_util.target.renderer._allDrawables);
//console.warn(this.last_util.target.renderer);
    let effectiveUtil = this.last_util;
    const hasValidLastUtil = !!(effectiveUtil && effectiveUtil.target && typeof effectiveUtil.target.direction !== "undefined");
    if (!hasValidLastUtil) {
      const robotTarget = this.runtime && Array.isArray(this.runtime.targets)
        ? this.runtime.targets.find(t => !t.isStage && t.sprite && t.sprite.name === 'Robbo Robot')
        : null;
      if (robotTarget) {
        effectiveUtil = {target: robotTarget};
      }
    }
    if (!(effectiveUtil && effectiveUtil.target && typeof effectiveUtil.target.direction !== "undefined")) {
      return -1;
    }
    return(this.robot_set_sens(effectiveUtil,index));
  }

  getSimTarget (util) {
    if (!this.runtime.sim_ac) return util.target;
    const robot = this.runtime.targets.find(t => !t.isStage && t.sprite && t.sprite.name === 'Robbo Robot');
    return robot || util.target;
  }

  getSimSensorDebugData () {
    if (!this.runtime || !this.runtime.sim_ac) return [];
    const robot = this.runtime.targets.find(t => !t.isStage && t.sprite && t.sprite.name === 'Robbo Robot');
    if (!robot) return [];
    return this.simSensorProbes.map((sensorCfg, idx) => {
      const ray = this.getSimSensorRay({target: robot}, idx);
      const startX = ray.startPoint[0];
      const startY = ray.startPoint[1];
      const directionRadians = ray.directionRadians;
      const endX = startX + 10 * Math.cos(directionRadians);
      const endY = startY + 10 * Math.sin(directionRadians);
      return {
        sensorIndex: idx + 1,
        startX: startX,
        startY: startY,
        endX: endX,
        endY: endY
      };
    });
  }

  getSimSensorRay (util, sensorIndex) {
    const sensorCfg = this.simSensorProbes[sensorIndex] || this.simSensorProbes[0];
    const forwardRadians = MathUtil.degToRad(90 - util.target.direction);
    const forwardX = Math.cos(forwardRadians);
    const forwardY = Math.sin(forwardRadians);
    const rightX = Math.cos(forwardRadians - Math.PI / 2);
    const rightY = Math.sin(forwardRadians - Math.PI / 2);

    const startX = util.target.x + (rightX * sensorCfg.localX) + (forwardX * sensorCfg.localY);
    const startY = util.target.y + (rightY * sensorCfg.localX) + (forwardY * sensorCfg.localY);
    const directionRadians = sensorCfg.direction === 'backward' ? (forwardRadians + Math.PI) : forwardRadians;
    return {
      startPoint: [startX, startY, 0],
      directionRadians: directionRadians
    };
  }

  robot_motors_on_for_seconds (args, util) {


      clearTimeout(this.robot_motors_on_for_seconds_end_timeout);
       this.robot_motors_on_for_seconds_end_timeout = setTimeout(() => {
          this.runtime.RCA.setRobotPower(0,0,0);
            clearInterval(this.sim_int);
            this.runtime.going=false;
       }, 40);

      this.is_motors_on_active = false;
      if (util.stackFrame.timer) {
          const timeElapsed = util.stackFrame.timer.timeElapsed();
          if (timeElapsed < util.stackFrame.duration * 1000) {
              util.yield();
          }
          else {
              if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){
                  this.runtime.RCA.block_A_CommandQueue();
                  util.yield();
                  return;
              }
              else{
                  util.stackFrame.timer = undefined;
                  clearTimeout(this.sim_int);
                  this.runtime.going=false;
                  clearTimeout(this.robot_motors_on_for_seconds_end_timeout);
                  this.runtime.RCA.setRobotPower(0,0,0);
                  this.runtime.RCA.unblock_A_CommandQueue();
              }
          }
      }
      else {
          if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){
                  this.runtime.RCA.block_A_CommandQueue();
                  util.yield();
                  return;
          }
          util.stackFrame.timer = new Timer();
          util.stackFrame.timer.start();
          util.stackFrame.duration = Cast.toNumber(args.SECONDS);
          if (util.stackFrame.duration <= 0) {
              return;
          }
          if(this.runtime.sim_ac){
              const simTarget = this.getSimTarget(util);
              this.runtime.going=true;
              this.xc=simTarget.x;
              this.yc=simTarget.y;
              simTarget.setXY(this.xc, this.yc);
              this.sim_int = setInterval(() => {
             const radians = MathUtil.degToRad(90 - simTarget.direction);
             let dist = (this.sim_pl+this.sim_pr)/2*this.kW;
             this.sim_dist_l+=Math.abs(this.sim_pl*this.kW);
             this.sim_dist_r+=Math.abs(this.sim_pr*this.kW);
             const dx = dist * Math.cos(radians);
             const dy = dist * Math.sin(radians);
             this.yc+=dy;
             this.xc+=dx;
             simTarget.setXY(this.xc, this.yc);
             if(simTarget.isTouchingColor(this.wall_color)){
               this.yc-=dy;
               this.xc-=dx;
               simTarget.setXY(this.xc, this.yc);
             }
             simTarget.setDirection(simTarget.direction + MathUtil.radToDeg(Math.atan((this.sim_pl-this.sim_pr)/this.rad)));
              }, this.fps);
          }
          else
          {
          clearInterval(this.motors_on_interval);
          this.runtime.RCA.setRobotPower(this.power_left,this.power_right,0);
          this.runtime.RCA.unblock_A_CommandQueue();
          }
          util.yield();
      }
  }

  robot_motors_on(args, util){

    //  console.trace("Trace: ");

    //  this.runtime.enableProfiling((frame) => {

    //         console.warn("frame: ");
    //         console.warn(frame);
    //     });




    if(this.runtime.sim_ac){
      const simTarget = this.getSimTarget(util);
      clearInterval(this.sim_int);
      this.runtime.going=true;
      this.xc=simTarget.x;
      this.yc=simTarget.y;
      simTarget.setXY(this.xc, this.yc);
      const radians = MathUtil.degToRad(90 - simTarget.direction);
      let dist = (this.sim_pl+this.sim_pr)/2*this.kW;
      const dx = dist * Math.cos(radians);
      const dy = dist * Math.sin(radians);
      this.yc+=dy;
      this.xc+=dx;
      simTarget.setXY(this.xc, this.yc);
      if(simTarget.isTouchingColor(this.wall_color)){
        this.yc-=dy;
        this.xc-=dx;
        simTarget.setXY(this.xc, this.yc);
      }
      simTarget.setDirection(simTarget.direction + MathUtil.radToDeg(Math.atan((this.sim_pl-this.sim_pr)/this.rad)));
      this.sim_int = setInterval(() => {
        const radians = MathUtil.degToRad(90 - simTarget.direction);
        let dist = (this.sim_pl+this.sim_pr)/2*this.kW;
        this.sim_dist_l+=Math.abs(this.sim_pl*this.kW);
        this.sim_dist_r+=Math.abs(this.sim_pr*this.kW);
        const dx = dist * Math.cos(radians);
        const dy = dist * Math.sin(radians);
        this.yc+=dy;
        this.xc+=dx;
        simTarget.setXY(this.xc, this.yc);
        if(simTarget.isTouchingColor(this.wall_color)){
          this.yc-=dy;
          this.xc-=dx;
          simTarget.setXY(this.xc, this.yc);
        }
        simTarget.setDirection(simTarget.direction + MathUtil.radToDeg(Math.atan((this.sim_pl-this.sim_pr)/this.rad)));
      }, this.fps);
  }
  else{
      let power_left = this.power_left;
      let power_right = this.power_right;
      clearTimeout(this.robot_motors_on_for_seconds_timeout_stop);
      clearInterval(this.motors_off_interval);
      clearInterval(this.motors_on_interval);
      this.need_to_stop = false;
      this.is_motors_on_active = true;
       this.motors_on_loop_need = true;
       this.command_sent = false;
      if (this.runtime.RCA.isRobotReadyToAcceptCommand()){
           this.runtime.RCA.setRobotPower(this.power_left,this.power_right,0);
           this.command_sent = true;
           this.runtime.RCA.unblock_A_CommandQueue();
           clearInterval(this.a_command_unblock_interval);
      }else{
         this.runtime.RCA.block_A_CommandQueue();
         util.yield();
      }
    }
  }

  robot_motors_off(args, util){
      clearInterval(this.sim_int);
      this.last_util=util;
     // console.warn(util);
     // console.warn(this.last_util);
      if(!this.runtime.sim_ac){
        this.runtime.going=false;
        clearInterval(this.motors_on_interval);
        clearInterval(this.motors_off_interval);
        this.need_to_stop = true;
        this.is_motors_on_active = false;
        if (this.runtime.RCA.isRobotReadyToAcceptCommand()){
           this.runtime.RCA.setRobotPower(0,0,0);
           this.command_sent = true;
           this.runtime.RCA.unblockPowerCommand();
           this.runtime.RCA.unblock_A_CommandQueue();
           clearInterval(this.a_command_unblock_interval);
        }else{
         this.runtime.RCA.block_A_CommandQueue();
         this.runtime.RCA.blockPowerCommand();
         util.yield();
        }
      }
  }


  update_power_using_direction(direction){

      // if ( this.runtime.are_motors_inverted){

      //   console.warn("Motors inverted ! ");
      // }

      switch (direction) {

        case "direction_forward":

          //  this.power = Math.round(this.power_in_percent * 0.63);
            this.power_left   =   Math.round(this.power_in_percent_left * 0.63);
            this.power_right  =   Math.round(this.power_in_percent_right * 0.63);
            this.sim_pl=Math.round(this.power_in_percent_left * 0.63);
            this.sim_pr=Math.round(this.power_in_percent_right * 0.63);

            if ( this.runtime.left_motor_inverted){

              //console.warn("Left motor inverted from forward to backward! ");

              this.power_left   =   Math.round(this.power_in_percent_left * 0.63) +  64;
             // this.power_right  =   Math.round(this.power_in_percent_right * 0.63) +  64;
            }

            if ( this.runtime.right_motor_inverted){

             // console.warn("Right motor inverted from forward to backward! ");

              //this.power_left   =   Math.round(this.power_in_percent_left * 0.63) +  64;
              this.power_right  =   Math.round(this.power_in_percent_right * 0.63) +  64;
            }


          break;

          case "direction_backward":

          //  this.power = Math.round(this.power_in_percent * 0.63) + 64;
            this.power_left   =   Math.round(this.power_in_percent_left * 0.63) +  64;
            this.power_right  =   Math.round(this.power_in_percent_right * 0.63) +  64;
            this.sim_pl=-Math.round(this.power_in_percent_left * 0.63);
            this.sim_pr=-Math.round(this.power_in_percent_right * 0.63);

            if ( this.runtime.left_motor_inverted){

             // console.warn("Left Motor inverted from backward to  forward! ");

              this.power_left   =   Math.round(this.power_in_percent_left * 0.63);
              //this.power_right  =   Math.round(this.power_in_percent_right * 0.63);
            }

            if ( this.runtime.right_motor_inverted){

             // console.warn("Right Motor inverted from backward to  forward! ");

              //this.power_left   =   Math.round(this.power_in_percent_left * 0.63);
              this.power_right  =   Math.round(this.power_in_percent_right * 0.63);
            }

          break;

          case "direction_left":

          //  this.power = Math.round(this.power_in_percent * 0.63);
            this.power_left     =   Math.round(this.power_in_percent_left * 0.63) +  64;
            this.power_right    =   Math.round(this.power_in_percent_right * 0.63);
            this.sim_pl=-Math.round(this.power_in_percent_left * 0.63);
            this.sim_pr=Math.round(this.power_in_percent_right * 0.63);

            if ( this.runtime.left_motor_inverted){

             // console.warn("Left Motor  inverted from left to  right! ");

              this.power_left   =   Math.round(this.power_in_percent_left * 0.63);
             
            }

            if ( this.runtime.right_motor_inverted){

             // console.warn("Right Motor inverted from left to  right! ");

             
              this.power_right  =   Math.round(this.power_in_percent_right * 0.63) + 64;
            }

          break;

          case "direction_right":

          //  this.power = Math.round(this.power_in_percent * 0.63);
            this.power_left   =   Math.round(this.power_in_percent_left * 0.63);
            this.power_right  =   Math.round(this.power_in_percent_right * 0.63) +  64;
            this.sim_pl=Math.round(this.power_in_percent_left * 0.63);
            this.sim_pr=-Math.round(this.power_in_percent_right * 0.63);

            if ( this.runtime.left_motor_inverted){

             // console.warn("Left Motor inverted from right to  left! ");

              this.power_left   =   Math.round(this.power_in_percent_left * 0.63) + 64;
             
            }

            if ( this.runtime.right_motor_inverted){

             // console.warn("Right Motor inverted from right to  left! ");

              this.power_right  =   Math.round(this.power_in_percent_right * 0.63);
            }

          break;
        default:
      }
    }

robot_first_draw(util){
  const renderer = util && util.target ? util.target.renderer : null;
  if (!renderer || typeof renderer._allDrawables === "undefined") return "no_drawable";

  const stageTarget = this.runtime && typeof this.runtime.getTargetForStage === "function" ?
    this.runtime.getTargetForStage() : null;
  if (stageTarget && typeof stageTarget.drawableID !== "undefined" && stageTarget.drawableID !== null) {
    const stageDrawable = renderer._allDrawables[stageTarget.drawableID];
    if (typeof stageDrawable !== "undefined" && stageDrawable != null) {
      return stageDrawable;
    }
  }

  for (let key in renderer._allDrawables) {
    if (typeof renderer._allDrawables[key] !== "undefined" && renderer._allDrawables[key] != null) {
      return renderer._allDrawables[key];
    }
  }
  return "no_drawable";
}

  sampleStageColor(util, pointVec3) {
    const drawable = this.robot_first_draw(util);
    if (drawable === "no_drawable") return [255, 255, 255];

    const renderer = util && util.target ? util.target.renderer : null;
    if (!renderer) return [255, 255, 255];

    const sampleFn =
      (renderer.constructor && typeof renderer.constructor.sampleColor3b === "function")
        ? renderer.constructor.sampleColor3b
        : null;
    if (!sampleFn) return [255, 255, 255];

    const out = [255, 255, 255];
    const drawableSampleArg = (drawable && typeof drawable === "object" && drawable.drawable)
      ? [drawable]
      : [{drawable: drawable}];
    try {
      sampleFn(pointVec3, drawableSampleArg, out);
      return out;
    } catch (e) {
      return [255, 255, 255];
    }
  }

  getDistToWall(util, radians, startPoint, maxDist) {
    const maxDistance = typeof maxDist === "number" ? maxDist : 100;
    const step = 1;
    for (let dist = 0; dist <= maxDistance; dist += step) {
      const point = [
        startPoint[0] + dist * Math.cos(radians),
        startPoint[1] + dist * Math.sin(radians),
        0
      ];
      const color = this.sampleStageColor(util, point);
      if (
        Math.abs(color[0] - this.wall_color[0]) <= 2 &&
        Math.abs(color[1] - this.wall_color[1]) <= 2 &&
        Math.abs(color[2] - this.wall_color[2]) <= 2
      ) {
        return dist;
      }
    }
    return maxDistance;
  }

  robot_set_direction_to(args, util){

        this.robot_direction = args.ROBOT_DIRECTION;
            this.update_power_using_direction(this.robot_direction);
            if(!this.runtime.sim_ac){
            if (this.is_motors_on_active){
                if (this.runtime.RCA.isRobotReadyToAcceptCommand()){
                   this.activate_robot_power();
                   this.runtime.RCA.unblock_A_CommandQueue();
                }else{
                    this.runtime.RCA.block_A_CommandQueue();
                    util.yield();
                }
            }
        }
  }


  activate_robot_power(){

        this.runtime.RCA.setRobotPower(this.power_left,this.power_right,0);

  }

  robot_touch(util,ray){
    this.ddx = 1.5 * Math.cos(ray.directionRadians);
    this.ddy = 1.5 * Math.sin(ray.directionRadians);
    this.ddp = [];this.ddp[0]=ray.startPoint[0]; this.ddp[1]=ray.startPoint[1]; this.ddp[2]=0;this.ddp[3]=0;;
    this.ddd = []; this.ddd[0]=this.robot_first_draw(util);
    this.ddl = []; this.ddl[0]=this.wall_color[0];this.ddl[1]=this.wall_color[1];this.ddl[2]=this.wall_color[2];
if(this.getDistToWall(util, ray.directionRadians, this.ddp, 100)>20)
  return 0;
return 100;
    }

  robot_get_dist(util,ray){
            this.ddx = 1.5 * Math.cos(ray.directionRadians);
            this.ddy = 1.5 * Math.sin(ray.directionRadians);
            this.ddp = [];this.ddp[0]=ray.startPoint[0]; this.ddp[1]=ray.startPoint[1]; this.ddp[2]=0;this.ddp[3]=0;;
            this.ddd = []; this.ddd[0]=this.robot_first_draw(util);
            this.ddl = []; this.ddl[0]=this.wall_color[0];this.ddl[1]=this.wall_color[1];this.ddl[2]=this.wall_color[2];

          return this.getDistToWall(util, ray.directionRadians, this.ddp, 100);
    }

  robot_set_sens(util,a){
      var radians=0,dx=0,dy=0;
      const ray = this.getSimSensorRay(util, a);
      // In simulation the sensor types are selected in GUI and applied to RCA via `setRobotSensor`.
      // The sim sensor model used to read from `runtime.sens_list`, but `runtime.sens_list` is never updated.
      // So we read the selected type from `runtime.RCA.sensors_array[a]` instead.
      const simSensorType =
        (this.runtime && this.runtime.RCA && this.runtime.RCA.sensors_array && this.runtime.RCA.sensors_array[a] != null)
          ? this.runtime.RCA.sensors_array[a]
          : this.runtime.sens_list[a];

      switch(simSensorType) // 0..7 mapping: nosensor/line/led/light/touch/proximity/ultrasonic/color
      {
      case 0: // nosensor
          return -1;
      break;
      case 1: // line
          var p=[];
          p[0]=ray.startPoint[0]; p[1]=ray.startPoint[1]; p[2]=0;
          var l= this.sampleStageColor(util, p);
          sensor_data= Math.round(l[0]+l[1]+l[2])/3;
          //    console.warn("GET2"+sensor_data);
            return sensor_data;
            break;
            case 2: // led
            return -1;
            break;
          case 7: // color
          var p=[];
          p[0]=ray.startPoint[0]; p[1]=ray.startPoint[1]; p[2]=0;
          var l= this.sampleStageColor(util, p);
          return l;
          break;
          case 4: // touch
          return this.robot_touch(util,ray);
          break;
          case 5: // proximity
          return Number(100-this.robot_get_dist(util,ray));
          break;
          case 6: // ultrasonic
          return this.robot_get_dist(util,ray);
          break;
          case 3: // light
          var p=[];p[0]=util.target.x; p[1]=util.target.y; p[2]=0;
          var l= this.sampleStageColor(util, p);
          sensor_data=255 - Math.round(l[0]+l[1]+l[2])/3;
          return sensor_data;
          break;
          default:
          console.log("WTF");
          return -1;
        }
  }

  robot_get_sensor_data(args, util){
          if (util && util.target && typeof util.target.direction !== "undefined") {
            this.last_util = util;
          }

          var sensor = args.ROBOT_SENSORS;
          var sensor_data = null;

          switch (sensor) {
            case "sensor1":
            //  console.warn("VM"+this.runtime.sens_list[0]);
              if(this.runtime.sim_ac){
              sensor_data = this.robot_set_sens(util,0);
              }
              else
              sensor_data = this.runtime.RCA.getSensorData(0);
              break;

           case "sensor2":

           if(this.runtime.sim_ac){
           sensor_data = this.robot_set_sens(util,1);
           }
           else
           sensor_data = this.runtime.RCA.getSensorData(1);
           break;

          case "sensor3":

          if(this.runtime.sim_ac){
          sensor_data = this.robot_set_sens(util,2);
          }
          else
          sensor_data = this.runtime.RCA.getSensorData(2);
          break;

          case "sensor4":

          if(this.runtime.sim_ac){
          sensor_data = this.robot_set_sens(util,3);
          }
          else
          sensor_data = this.runtime.RCA.getSensorData(3);
          break;

         case "sensor5":

         if(this.runtime.sim_ac){
         sensor_data = this.robot_set_sens(util,4);
         }
         else
         sensor_data = this.runtime.RCA.getSensorData(4);
         break;

        case "sensor_trip_meter_left":
              if(this.runtime.sim_ac)
              sensor_data = Math.round(this.sim_dist_l);
              else
              sensor_data = this.runtime.RCA.getLeftPath();

            break;

        case "sensor_trip_meter_right":
              if(this.runtime.sim_ac)
              sensor_data = Math.round(this.sim_dist_r);
              else
              sensor_data = this.runtime.RCA.getRightPath();

            break;



            default:

            sensor_data = -1;

          }

      return sensor_data;

    }

  robot_get_rgb_sensor_data(args,util){

      //    console.log(`robot_get_rgb_sensor_data   sensor: ${args.ROBOT_SENSORS_FOR_RGB} color: ${args.RGB_VALUES} `);

      let sensor_id = Number(args.ROBOT_SENSORS_FOR_RGB.replace("sensor","")) - 1;

      if(this.runtime.sim_ac){
        var p=[];  p[0]=util.target.x; p[1]=util.target.y; p[2]=0;
        var l= this.sampleStageColor(util, p);
  //      console.warn("result is"+l);
        switch (args.RGB_VALUES) {

          case "red":

                return l[0];

            break;

        case "green":

                return l[1];

          break;

        case "blue":

                  return l[2];

            break;

          default:

                return -1; // TODO: правильно обрабатывать

        }

      }
      else
       {
        let rgb_array = this.runtime.RCA.getColorCorrectedRawValues(sensor_id);
          switch (args.RGB_VALUES) {

            case "red":

                  return rgb_array[0];

              break;

          case "green":

                  return rgb_array[1];

            break;

          case "blue":

                    return rgb_array[2];

              break;

            default:

                  return -1; // TODO: правильно обрабатывать

          }
        }

    }

  robot_is_current_color(args){

    //   console.log(`robot_is_current_color   sensor: ${args.ROBOT_SENSORS_FOR_RGB} color: ${args.COLORS} `);

       let sensor_id = Number(args.ROBOT_SENSORS_FOR_RGB.replace("sensor","")) - 1;

       let color = args.COLORS;

       let current_color = this.runtime.RCA.colorFilter(sensor_id,true);

       if ((color == "unknown") && (Array.isArray(current_color))){

          return true;

       }else if (color == current_color){

           return true;

       }else{

         return false;

       }

    }

  check_value_out_of_range(value,low,high){

        return (value > high)?high:((value < low)?low:value);

  }

  robot_set_motors_left_right_power_and_direction_separately(args, util){

                     // this.runtime.RCA.setRobotPower(0,0,0);
    this.power_in_percent_left  =   (args.POWER_LEFT > 100)?100:((args.POWER_LEFT < 0)?0:args.POWER_LEFT);
    this.power_in_percent_right =   (args.POWER_RIGHT > 100)?100:((args.POWER_RIGHT < 0)?0:args.POWER_RIGHT);

    var robot_left_motor_direction = args.ROBOT_LEFT_MOTOR_DIRECTION;

    switch (robot_left_motor_direction) {

      case "direction_forward":


          this.power_left   =   Math.round(this.power_in_percent_left * 0.63);

          if ( this.runtime.left_motor_inverted){

           // console.warn("Left motor inverted from forward to backward! ");

            this.power_left   =   Math.round(this.power_in_percent_left * 0.63) +  64;
          }



        break;

        case "direction_backward":


          this.power_left   =   Math.round(this.power_in_percent_left * 0.63) +  64;
          this.sim_pl=-Math.round(this.power_in_percent_left * 0.63);

          if ( this.runtime.left_motor_inverted){

           // console.warn("Left Motor inverted from backward to  forward! ");

            this.power_left   =   Math.round(this.power_in_percent_left * 0.63);
          }

        break;


      default:

    }

    var robot_right_motor_direction = args.ROBOT_RIGHT_MOTOR_DIRECTION;

    switch (robot_right_motor_direction) {

      case "direction_forward":


          this.power_right   =   Math.round(this.power_in_percent_right * 0.63);
          this.sim_pr=Math.round(this.power_in_percent_right * 0.63);

          if ( this.runtime.right_motor_inverted){

           // console.warn("Right motor inverted from forward to backward! ");

            this.power_right  =   Math.round(this.power_in_percent_right * 0.63) +  64;
          }

        break;

      case "direction_backward":


          this.power_right   =   Math.round(this.power_in_percent_right * 0.63) +  64;
          this.sim_pr=-Math.round(this.power_in_percent_right * 0.63);

          if ( this.runtime.right_motor_inverted){

           // console.warn("Right Motor inverted from backward to  forward! ");

            this.power_right  =   Math.round(this.power_in_percent_right * 0.63);
          }

        break;


      default:

    }

     if (this.is_motors_on_active){

                if (this.runtime.RCA.isRobotReadyToAcceptCommand()){


                   this.activate_robot_power();
                   this.runtime.RCA.unblock_A_CommandQueue();


                  }else{

                    this.runtime.RCA.block_A_CommandQueue();
                    util.yield();


                  }

            }


    }

  check_65535(steps){

          return (steps > 65535)?65535:((steps < 0)?0:steps);

  }

  calculate_steps_delta(){

        let leftPath = this.runtime.RCA.getLeftPath();
        let rightPath = this.runtime.RCA.getRightPath();

        return  ((leftPath > rightPath)?leftPath:rightPath )  - this.stepsInitValue;



      //  return  this.runtime.RCA.getLeftPath()  - this.stepsInitValue; // TODO: проверить на 65535

  }

  calculate_steps_delta_left(){



            return  this.runtime.RCA.getLeftPath()  - this.stepsInitValueLeft; // TODO: проверить на 65535

  }

  calculate_steps_delta_right(){



      return  this.runtime.RCA.getRightPath()  - this.stepsInitValueRight; // TODO: проверить на 65535

  }

  robot_motors_on_for_steps(args, util){
    this.is_motors_on_active = false;

      clearTimeout(this.robot_motors_on_for_seconds_timeout_stop);
      if ((util.stackFrame.steps != null) && (typeof(util.stackFrame.steps) != 'undefined') ) {
        var stepsDeltaLeft  =  this.calculate_steps_delta_left();
        var stepsDeltaRight =  this.calculate_steps_delta_right();

        if ((stepsDeltaLeft < util.stackFrame.steps  ) && (stepsDeltaRight < util.stackFrame.steps) && (!this.need_to_stop) && (this.distl<Number(args.STEPS) && this.distr<Number(args.STEPS))) {  // TODO: сделать корректную проверку для робота без энкодеров
            util.yield();
          } else{

                util.stackFrame.steps = null;
                clearInterval(this.sim_int);
                this.distl=0;
                this.distr=0;
                this.runtime.going=false;
          }
      } else {
            clearInterval(this.motors_on_interval);
            clearInterval(this.sim_int);
            this.runtime.going=false;
            if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){
                this.runtime.RCA.block_A_CommandQueue();
                util.yield();
                return;
            }else{
               this.runtime.RCA.unblock_A_CommandQueue();
            }
          util.stackFrame.steps = this.check_65535(args.STEPS);
          this.stepsInitValueLeft  =  this.runtime.RCA.getLeftPath();
          this.stepsInitValueRight =  this.runtime.RCA.getRightPath();
          if (util.stackFrame.steps <= 0) {
              return;
          }
          this.need_to_stop = false;
          if(this.runtime.sim_ac){
            const simTarget = this.getSimTarget(util);
            clearInterval(this.sim_int);
            this.runtime.going=true;
            this.xc=simTarget.x;
            this.yc=simTarget.y;
            simTarget.setXY(this.xc, this.yc);
            this.distl=0;
            this.distr=0;
            this.sim_int = setInterval(() => {
             const radians = MathUtil.degToRad(90 - simTarget.direction);
             let dist = (this.sim_pl+this.sim_pr)/2*this.kW;
             this.sim_dist_l+=Math.abs(this.sim_pl*this.kW);
             this.sim_dist_r+=Math.abs(this.sim_pr*this.kW);
             this.distl+=Math.abs(this.sim_pl*this.kW);
             this.distr+=Math.abs(this.sim_pr*this.kW);
             const dx = dist * Math.cos(radians);
             const dy = dist * Math.sin(radians);
             this.yc+=dy;
             this.xc+=dx;
             simTarget.setXY(this.xc, this.yc);
             if(simTarget.isTouchingColor(this.wall_color)){
               this.yc-=dy;
               this.xc-=dx;
               simTarget.setXY(this.xc, this.yc);
             }
             simTarget.setDirection(simTarget.direction + MathUtil.radToDeg(Math.atan((this.sim_pl-this.sim_pr)/this.rad)));
              }, this.fps);
          }
          else {
            this.runtime.RCA.setRobotPowerAndStepLimits(this.power_left,this.power_right,util.stackFrame.steps,0);
          }
          util.yield();
      }




    }

  robot_turnright(args, util){
    this.is_motors_on_active = false;
    clearTimeout(this.robot_motors_on_for_seconds_timeout_stop);
    if ((util.stackFrame.steps != null) && (typeof(util.stackFrame.steps) != 'undefined')) {
      var stepsDeltaLeft  =  this.calculate_steps_delta_left();
      var stepsDeltaRight =  this.calculate_steps_delta_right();
      if (  (stepsDeltaLeft < util.stackFrame.steps  ) && (stepsDeltaRight < util.stackFrame.steps)  && (!this.need_to_stop)&& (this.distl<Number(args.DEGREES)/230*100/5.636 && this.distr<Number(args.DEGREES)/230*100/5.636) ) { // TODO: сделать корректную проверку для робота без энкодеров
          util.yield();
        } else{
            clearInterval(this.sim_int);
            this.runtime.going=false;
            util.stackFrame.steps = null;
            /*console.warn(this.start_deg);
            console.warn(Number(args.DEGREES));
            console.warn(this.start_deg+Number(args.DEGREES));*/
            util.target.setDirection(this.start_deg+Number(args.DEGREES));
      }
    } else {
           if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){
                this.runtime.RCA.block_A_CommandQueue();
                util.yield();
                return;
            }else{
               this.runtime.RCA.unblock_A_CommandQueue();
            }
        util.stackFrame.steps = this.check_65535(Math.round(args.DEGREES / DEGREE_RATIO))
        this.stepsInitValueLeft  =  this.runtime.RCA.getLeftPath();
        this.stepsInitValueRight =  this.runtime.RCA.getRightPath();
        clearInterval(this.sim_int);
        this.runtime.going=false;
        clearInterval(this.motors_on_interval);
      if (util.stackFrame.steps <= 0) {
            return;
        }
        let power_left =   Math.round(30 * 0.63);
        let power_right =  Math.round(30 * 0.63) + 64;
       
        // if ( this.runtime.are_motors_inverted){

        //   console.warn("robot_turnright: Motors inverted from right to  left! ");

        //   power_left   =   Math.round(30 * 0.63) + 64;
        //   power_right  =   Math.round(30 * 0.63);
        // }

        if ( this.runtime.left_motor_inverted){

         // console.warn("Left Motor inverted from right to  left! ");

          power_left   =   Math.round(30 * 0.63) + 64;
         
        }

        if ( this.runtime.right_motor_inverted){

         // console.warn("Right Motor inverted from right to  left! ");

          power_right  =   Math.round(30 * 0.63);
        }

        clearInterval(this.sim_int);

        if(this.runtime.sim_ac){
  //        console.warn(this.sim_pr);
          this.runtime.going=true;
        this.start_deg = util.target.direction;
          let simpl=30;
          let simpr=-30;
          this.distl=0;
          this.distr=0;
          this.xc=util.target.x;
          this.yc=util.target.y;
          this.sim_int = setInterval(() => {
          const radians = MathUtil.degToRad(90 - util.target.direction);
          let dist = (simpl+simpr)/2*this.kW;
          this.sim_dist_l+=Math.abs(simpl*this.kW);
          this.sim_dist_r+=Math.abs(simpr*this.kW);
          this.distl+=Math.abs(simpl*this.kW);
          this.distr+=Math.abs(simpr*this.kW);
          const dx = dist * Math.cos(radians);
          const dy = dist * Math.sin(radians);
          this.yc+=dy;
          this.xc+=dx;
          util.target.setXY(this.xc, this.yc);
          util.target.setDirection(util.target.direction + MathUtil.radToDeg(Math.atan((simpl-simpr)/this.rad)));
          }, this.fps);
        }
        else{
        this.runtime.RCA.setRobotPowerAndStepLimits(power_left,power_right, util.stackFrame.steps ,0);
        }
        this.need_to_stop = false;
        util.yield();
    }
    }

  robot_turnleft(args, util){

          this.is_motors_on_active = false;

          clearTimeout(this.robot_motors_on_for_seconds_timeout_stop);

          if ((util.stackFrame.steps != null) && (typeof(util.stackFrame.steps) != 'undefined') ) {

            //  const stepsDelta =  this.calculate_steps_delta();

            var stepsDeltaLeft  =  this.calculate_steps_delta_left();
            var stepsDeltaRight =  this.calculate_steps_delta_right();

            if (  (stepsDeltaLeft < util.stackFrame.steps  ) && (stepsDeltaRight < util.stackFrame.steps)  && (!this.need_to_stop)&& (this.distl<Number(args.DEGREES)/230*100/5.636 && this.distr<Number(args.DEGREES)/230*100/5.636)  ) { // TODO: сделать корректную проверку для робота без энкодеров

                util.yield();

              } else{

              //      console.log(`robot_turnleft exit function stepsDeltaLeft: ${stepsDeltaLeft} stepsDeltaRight: ${stepsDeltaRight}`);
                    clearInterval(this.sim_int);
                    util.stackFrame.steps = null;
                    this.runtime.going=false;
                    util.target.setDirection(this.start_deg-Number(args.DEGREES));
              }
          } else {

              if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){

                this.runtime.RCA.block_A_CommandQueue();
                util.yield();
                return;

              }else{

               this.runtime.RCA.unblock_A_CommandQueue();

              }
              clearInterval(this.sim_int);
              this.runtime.going=false;
              util.stackFrame.steps = this.check_65535(Math.round(args.DEGREES / DEGREE_RATIO))
              this.stepsInitValueLeft  =  this.runtime.RCA.getLeftPath();
              this.stepsInitValueRight =  this.runtime.RCA.getRightPath();
            //  util.stackFrame.steps_counter = 0;


              clearInterval(this.motors_on_interval);
              // //  clearInterval(this.motors_off_interval);
            //  this.runtime.RCA.setRobotPower(0,0,0);

              if (util.stackFrame.steps <= 0) {

                  return;
              }

              let power_left =   Math.round(30 * 0.63) + 64;
              let power_right =  Math.round(30 * 0.63);

              // if ( this.runtime.are_motors_inverted){

              //   console.warn("robot_turnleft: Motors inverted from left to right! ");
      
              //   power_left   =   Math.round(30 * 0.63);
              //   power_right  =   Math.round(30 * 0.63) + 64;
              // }

              if ( this.runtime.left_motor_inverted){

               // console.warn("Left Motor  inverted from left to  right! ");
  
                power_left   =   Math.round(30 * 0.63);
               
              }
  
              if ( this.runtime.right_motor_inverted){
  
               // console.warn("Right Motor inverted from left to  right! ");
  
               
                power_right  =   Math.round(30 * 0.63) + 64;
              }


              if(this.runtime.sim_ac){
                  clearInterval(this.sim_int);
                    this.start_deg = util.target.direction;
                  this.runtime.going=true;
                  let simpl=-30;
                  let simpr=30;
                  this.distl=0;
                  this.distr=0;
                  this.xc=util.target.x;
                  this.yc=util.target.y;
                this.sim_int = setInterval(() => {
                 const radians = MathUtil.degToRad(90 - util.target.direction);
                 let dist = (simpl+simpr)/2*this.kW;
                 this.sim_dist_l+=Math.abs(simpl*this.kW);
                 this.sim_dist_r+=Math.abs(simpr*this.kW);
                 this.distl+=Math.abs(simpl*this.kW);
                 this.distr+=Math.abs(simpr*this.kW);
                 const dx = dist * Math.cos(radians);
                 const dy = dist * Math.sin(radians);
                 this.yc+=dy;
                 this.xc+=dx;
                 util.target.setXY(this.xc, this.yc);
                 util.target.setDirection(util.target.direction + MathUtil.radToDeg(Math.atan((simpl-simpr)/this.rad)));
                  }, this.fps);
              }
              else{
              this.runtime.RCA.setRobotPowerAndStepLimits(power_left,power_right, util.stackFrame.steps ,0);
              }
              this.need_to_stop = false;
              util.yield();
          }

    }

  robot_set_motors_power(args, util){

      let power = this.check_value_out_of_range(args.POWER,0,100);
        this.power_in_percent_left    =   power;
        this.power_in_percent_right   =   power;
        this.update_power_using_direction(this.robot_direction);
         if (this.is_motors_on_active){

                if (this.runtime.RCA.isRobotReadyToAcceptCommand()){


                   this.activate_robot_power();
                   this.runtime.RCA.unblock_A_CommandQueue();


                  }else{

                    this.runtime.RCA.block_A_CommandQueue();
                    util.yield();


                  }

            }


    }

  robot_set_motors_power_left_right_separately(args, util){

                   //   this.runtime.RCA.setRobotPower(0,0,0);
      this.power_in_percent_left    =   this.check_value_out_of_range(args.POWER_LEFT,0,100);
      this.power_in_percent_right   =   this.check_value_out_of_range(args.POWER_RIGHT,0,100);

    //  console.log(`robot_set_motors_power_left_right_separately power_in_percent_left: ${this.power_in_percent_left} power_in_percent_right: ${this.power_in_percent_right}`);

      this.update_power_using_direction(this.robot_direction);

       if (this.is_motors_on_active){

                if (this.runtime.RCA.isRobotReadyToAcceptCommand()){


                   this.activate_robot_power();
                   this.runtime.RCA.unblock_A_CommandQueue();


                  }else{

                    this.runtime.RCA.block_A_CommandQueue();
                    util.yield();


                  }

            }


    }

  robot_start_button_pressed(args, util){

        return (this.runtime.RCA.getButtonStartPushed() == 'true')?true:false;

  }

  robot_turn_led_on(args, util){

    //   console.log(`robot_turn_led_on led_position: ${args.ROBOT_POSITION}`);

       if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){

              this.runtime.RCA.block_A_CommandQueue();  //not ready right now
              util.yield();
              return;


        }


        //here we ready

        this.runtime.RCA.unblock_A_CommandQueue();

       switch (args.ROBOT_POSITION) {

         case 'position1':

                  this.runtime.RCA.turnLedOn(0,0);

           break;

          case 'position2':

                  this.runtime.RCA.turnLedOn(1,0);

             break;

          case 'position3':

                  this.runtime.RCA.turnLedOn(2,0);

            break;

          case 'position4':

                  this.runtime.RCA.turnLedOn(3,0);

            break;

        case 'position5':

                  this.runtime.RCA.turnLedOn(4,0);

         break;

         default:

       }

    }

  robot_turn_led_off(args, util){

    //  console.log(`robot_turn_led_off led_position: ${args.ROBOT_POSITION}`);

      if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){

              this.runtime.RCA.block_A_CommandQueue();  //not ready right now
              util.yield();
              return;


        }


        //here we ready

      switch (args.ROBOT_POSITION) {

        case 'position1':

                 this.runtime.RCA.turnLedOff(0,0);

          break;

         case 'position2':

                 this.runtime.RCA.turnLedOff(1,0);

            break;

         case 'position3':

                 this.runtime.RCA.turnLedOff(2,0);

           break;

         case 'position4':

                 this.runtime.RCA.turnLedOff(3,0);

           break;

       case 'position5':

                 this.runtime.RCA.turnLedOff(4,0);

        break;

        default:

      }

      this.runtime.RCA.unblock_A_CommandQueue();


    }

  robot_reset_trip_meters(args, util){

      if(this.runtime.sim_ac)
      {this.sim_dist_l=0;
      this.sim_dist_r=0;}
      else
      this.runtime.RCA.resetTripMeters();

  }

  robot_claw_closed(args, util){

  //    console.log(`robot_claw_closed degrees: ${args.CLAW_CLOSED_PERCENT}`);

        var degrees = this.check_value_out_of_range(args.CLAW_CLOSED_PERCENT,0,100);

        if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){

              this.runtime.RCA.block_A_CommandQueue();  //not ready right now
              util.yield();
              return;


        }


        //here we ready
         this.runtime.RCA.setClawDegrees(degrees,0);

        this.runtime.RCA.unblock_A_CommandQueue();

    }

  robot_claw_state(args, util){

    //    console.log(`robot_claw_state state: ${args.CLAW_STATES}`);

       if (!this.runtime.RCA.isRobotReadyToAcceptCommand()){

              this.runtime.RCA.block_A_CommandQueue();  //not ready right now
              util.yield();
              return;


        }


        //here we ready

        switch (args.CLAW_STATES) {

          case 'claw_open':

            this.runtime.RCA.setClawDegrees(0,0);

            break;

          case 'claw_half_open':

              this.runtime.RCA.setClawDegrees(50,0);

              break;

          case 'claw_closed':

              this.runtime.RCA.setClawDegrees(100,0);

              break;

          default:

        }

        this.runtime.RCA.unblock_A_CommandQueue();



    }

  robot_wall_color(args){
          const maskColor = Cast.toRgbColorList(args.COLOR);
          this.wall_color[0] = Cast.toNumber(maskColor[0]);
          this.wall_color[1] = Cast.toNumber(maskColor[1]);
          this.wall_color[2] = Cast.toNumber(maskColor[2]);
    }

  }

    module.exports = Scratch3RobotBlocks;
