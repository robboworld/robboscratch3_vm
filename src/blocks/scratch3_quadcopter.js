const Cast = require('../util/cast');
const MathUtil = require('../util/math-util');
const Timer = require('../util/timer');

const COPTER_SPRITE_NAME = 'Robbo Quadcopter';
const COSTUME_IDLE = 0;
const COSTUME_FLYING = 1;
const SIM_SCALE = 200;       // 1 meter = 200 Scratch pixels
const SIM_BASE_SIZE = 100;   // sprite size (%) at z=0
const SIM_SIZE_PER_METER = 30; // +30% per meter of altitude
const SIM_STEP_MS = 50;      // physics tick interval
const SIM_MOVE_SPEED = 0.4;  // meters per second for move_to_coord interpolation

class Scratch3QuadcopterBlocks {
    constructor (runtime) {
        /**
         * The runtime instantiating this block package.
         * @type {Runtime}
         */
        this.runtime = runtime;
          this.x = this.runtime.QCA.get_coord("X");
          this.nowx=0;
          this.y =this.runtime.QCA.get_coord("Y");
          this.nowy=0;
          this.z = this.runtime.QCA.get_coord("Z");
          this.nowz=0;
          this.defz= 0.3;
          this.yaw =this.runtime.QCA.get_coord("W");
          this.noww=0;
          this.dir = 0;
          this.fack = 0;
          this.delta= 0.01;
          this.speed= 1;

          this.yielded_time_start = Date.now();
          this.yielded_time_now   = Date.now();
          this.yielded_max_time = 1 * 60 * 1000;

          this.x_telemetry_delta = 0;
          this.y_telemetry_delta = 0;

          // --- Simulator state ---
          this.runtime.sim_copter_ac = false;
          this.runtime._copterSimBlocks = this;
          this.sim_x = 0;
          this.sim_y = 0;
          this.sim_z = 0;
          this.sim_yaw = 0;
          this.sim_is_flying = false;
          this.sim_battery = 100;
          this.sim_interval = null;
          this._sim_prev_flying = false;
    }

    getPrimitives () {
        return {
            copter_fly_up: this.copter_fly_up,
            copter_land:this.copter_land,
            copter_stop:this.copter_stop,
            copter_status:this.copter_status,
            copter_fly_distance:this.copter_fly_distance,
            copter_fly_time:this.copter_fly_time,
            copter_fly_for_time_with_speed:this.copter_fly_for_time_with_speed,
            copter_change_x_by:this.copter_change_x_by,
            copter_change_y_by:this.copter_change_y_by,
            copter_change_z_by:this.copter_change_z_by,
            copter_x_coord:this.copter_x_coord,
            copter_y_coord:this.copter_y_coord,
            copter_z_coord:this.copter_z_coord,
            copter_yaw:this.copter_yaw,
            copter_fly_for_seconds_to_coords:this.copter_fly_for_seconds_to_coords,
            copter_fly_to_coords:this.copter_fly_to_coords,
            copter_rotate:this.copter_rotate,
            copter_set_direction:this.copter_set_direction,
            copter_direction:this.copter_direction
        };
    }

    getMonitored () {
        return {};
    }

    // ===== Simulator helpers =====

    _getSimCopterTarget () {
        if (!this.runtime || !this.runtime.targets) return null;
        for (let i = 0; i < this.runtime.targets.length; i++) {
            const t = this.runtime.targets[i];
            if (t.isOriginal && !t.isStage && t.sprite && t.sprite.name === COPTER_SPRITE_NAME) {
                return t;
            }
        }
        return null;
    }

    _simApplyState () {
        const target = this._getSimCopterTarget();
        if (!target) return;
        target.setXY(this.sim_x * SIM_SCALE, this.sim_y * SIM_SCALE);
        target.setDirection(90 - this.sim_yaw);
        const sz = SIM_BASE_SIZE + this.sim_z * SIM_SIZE_PER_METER;
        target.setSize(sz);
        if (this.sim_is_flying !== this._sim_prev_flying) {
            this._sim_prev_flying = this.sim_is_flying;
            const costumeIdx = this.sim_is_flying ? COSTUME_FLYING : COSTUME_IDLE;
            if (target.currentCostume !== costumeIdx) {
                target.setCostume(costumeIdx);
            }
        }
    }

    _simClearInterval () {
        if (this.sim_interval) {
            clearInterval(this.sim_interval);
            this.sim_interval = null;
        }
    }

    /**
     * Animate sim copter toward target coords. Resolves when close enough or timeout.
     * Uses the same fack/yield pattern as the hardware path.
     */
    _simStartMoveToCoord (tx, ty, tz, tyaw) {
        this._simClearInterval();
        this.sim_interval = setInterval(() => {
            const dx = tx - this.sim_x;
            const dy = ty - this.sim_y;
            const dz = tz - this.sim_z;
            const step = SIM_MOVE_SPEED * (SIM_STEP_MS / 1000);
            const dist2d = Math.sqrt(dx * dx + dy * dy);
            if (dist2d > step) {
                this.sim_x += (dx / dist2d) * step;
                this.sim_y += (dy / dist2d) * step;
            } else {
                this.sim_x = tx;
                this.sim_y = ty;
            }
            if (Math.abs(dz) > step) {
                this.sim_z += Math.sign(dz) * step;
            } else {
                this.sim_z = tz;
            }
            // yaw interpolation
            let dyaw = tyaw - this.sim_yaw;
            if (dyaw > 180) dyaw -= 360;
            if (dyaw < -180) dyaw += 360;
            const yawStep = 90 * (SIM_STEP_MS / 1000); // 90 deg/s
            if (Math.abs(dyaw) > yawStep) {
                this.sim_yaw += Math.sign(dyaw) * yawStep;
            } else {
                this.sim_yaw = tyaw;
            }
            this.sim_yaw = this._castYawTo360(this.sim_yaw);
            this._simApplyState();
        }, SIM_STEP_MS);
    }

    _simIsAtTarget (tx, ty, tz, tyaw) {
        const posDelta = 0.02;
        const yawDelta = 3;
        let dyaw = Math.abs(tyaw - this.sim_yaw);
        if (dyaw > 180) dyaw = 360 - dyaw;
        return Math.abs(this.sim_x - tx) < posDelta &&
               Math.abs(this.sim_y - ty) < posDelta &&
               Math.abs(this.sim_z - tz) < posDelta &&
               dyaw < yawDelta;
    }

    _castYawTo360 (yaw) {
        if (yaw > 0) {
            yaw = yaw - ((yaw / 360) >> 0) * 360;
        } else if (yaw < 0) {
            yaw = yaw + ((yaw / 360) >> 0) * -360;
        }
        return yaw;
    }

    // ===== Block primitives =====

    copter_fly_up (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                this.sim_z = this.sim_z + 0.3;
                this.sim_is_flying = true;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) {
                    this.fack = 2;
                }
                if (this._simIsAtTarget(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw)) {
                    this.fack = 2;
                }
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path (original) ---
        if(this.fack==0)
        {
        this.init_start_coordinates();
        this.z = this.z + 0.3;
        console.log(`copter_fly_up: ${this.z}`);
        this.SendCordInterval =  setInterval(() =>{
          if (this.runtime.QCA.isQuadcopterConnected()){
                this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
          }else{
                clearInterval(this.SendCordInterval);
                this.fack=0
          }
        },100);
        this.yielded_time_start = Date.now();
        this.yielded_time_now = Date.now();
        this.fack = 1;
        util.yield();
        return;
        }
        else if(this.fack != 2)
        {
          if ((this.yielded_time_now - this.yielded_time_start ) >= this.yielded_max_time){
            this.fack=2;
          }
          this.nowz = this.runtime.QCA.get_coord("Z");
          if(Math.abs(this.nowz-this.z)<this.delta)
          this.fack=2;
          this.yielded_time_now = Date.now();
          util.yield();
          return;
        }
        clearInterval(this.SendCordInterval);
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    copter_land (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_z -= 0.1 * (SIM_STEP_MS / 200);
                    if (this.sim_z <= 0.02) {
                        this.sim_z = 0;
                        this.sim_is_flying = false;
                        this._simClearInterval();
                        this.fack = 2;
                    }
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) {
                    this.fack = 2;
                    this.sim_z = 0;
                    this.sim_is_flying = false;
                    this._simClearInterval();
                    this._simApplyState();
                }
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        this.init_start_coordinates();
        this.CopterLANDING =  setInterval(() =>{this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
          this.z-=0.1;
          if(this.z<=0.1){
             clearInterval(this.CopterLANDING);
              this.runtime.QCA.copter_land();
           }     },200)
    }

    copter_stop () {
        if (this.runtime.sim_copter_ac) {
            this._simClearInterval();
            this.sim_is_flying = false;
            this.sim_z = 0;
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if (this.SendCordInterval) {
            clearInterval(this.SendCordInterval);
            this.SendCordInterval = null;
        }
        if (this.CopterLANDING) {
            clearInterval(this.CopterLANDING);
            this.CopterLANDING = null;
        }
        this.runtime.QCA.copter_land();
    }

    copter_status (args, util) {
        if (this.runtime.sim_copter_ac) {
            return 1;
        }
        return(this.runtime.QCA.isQuadcopterConnected());
    }

    copter_fly_distance (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                const meters = Number(args.METERS);
                const rad = (this.sim_yaw + this.dir) * Math.PI / 180;
                const tx = this.sim_x + meters * Math.cos(rad);
                const ty = this.sim_y + meters * Math.sin(rad);
                this._sim_target_x = tx;
                this._sim_target_y = ty;
                this._simStartMoveToCoord(tx, ty, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this._sim_target_x, this._sim_target_y, this.sim_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          this.init_start_coordinates();
          this.x = this.x + Number(args.METERS) * Math.cos((this.yaw+this.dir) * Math.PI / 180);
          this.y = this.y + Number(args.METERS) * Math.sin((this.yaw+this.dir) * Math.PI / 180);
          console.log(`HUUUUIX: ${this.x}`);
          console.log(`HUUUUIY: ${this.y}`)
          this.yielded_time_start = Date.now();
          this.yielded_time_now = Date.now();
          this.SendCordInterval =  setInterval(() =>{
            if (this.runtime.QCA.isQuadcopterConnected()){
                  this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
            }else{
                  clearInterval(this.SendCordInterval);
                  this.fack=0;
            }
          },100);
          this.fack=1;
          util.yield();
          return;
        }
        else if(this.fack == 1)
        {
          this.nowx= this.runtime.QCA.get_coord("X");
          this.nowy=this.runtime.QCA.get_coord("Y");
          if((Math.abs(this.nowx-this.x)<this.delta) && (Math.abs(this.nowy-this.y)<this.delta))
          {
            this.fack=2;
          }
          else {
            if ((this.yielded_time_now - this.yielded_time_start ) >= this.yielded_max_time){
              this.fack=2;
            }
            this.yielded_time_now = Date.now();
            util.yield();
            return;
          }
        }
        clearInterval(this.SendCordInterval);
        this.fack=0;
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
    }

    copter_fly_time (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                const vx = this.speed * Math.cos((this.sim_yaw + this.dir) * Math.PI / 180);
                const vy = this.speed * Math.sin((this.sim_yaw + this.dir) * Math.PI / 180);
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_x += vx * (SIM_STEP_MS / 1000);
                    this.sim_y += vy * (SIM_STEP_MS / 1000);
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.fack = 1;
                const time_to_fly = Number(args.SECONDS) * 1000;
                setTimeout(() => { this.fack = 2; }, time_to_fly);
                util.yield();
                return;
            } else if (this.fack !== 2) {
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          let vx =  this.speed* Math.cos((this.yaw+this.dir) * Math.PI / 180);
          let vy =  this.speed* Math.sin((this.yaw+this.dir) * Math.PI / 180);
          this.runtime.QCA.move_with_speed(vx,vy,0,this.z);
          this.fack=1;
          let time_to_fly = Number(args.SECONDS)*1000;
          setTimeout(() => { this.fack = 2; }, time_to_fly);
          util.yield();
          return;
        }
        else if ( this.fack != 2)
        {
          util.yield();
          return;
        }
        this.x=this.runtime.QCA.get_coord("X");
        this.y=this.runtime.QCA.get_coord("Y");
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    copter_fly_for_time_with_speed (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                const vx = Number(args.X_SPEED);
                const vy = Number(args.Y_SPEED);
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_x += vx * (SIM_STEP_MS / 1000);
                    this.sim_y += vy * (SIM_STEP_MS / 1000);
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.fack = 1;
                const time_to_fly = Number(args.SECONDS) * 1000;
                setTimeout(() => { this.fack = 2; }, time_to_fly);
                util.yield();
                return;
            } else if (this.fack !== 2) {
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          let vx =  Number(args.X_SPEED);
          let vy =  Number(args.Y_SPEED);
          this.z = this.runtime.QCA.get_coord("Z");
          this.runtime.QCA.move_with_speed(vx,vy,0,this.z);
          this.fack=1;
          let time_to_fly = Number(args.SECONDS*1000);
          setTimeout(() => { this.fack = 2; }, time_to_fly);
          util.yield();
          return;
        }
        else if ( this.fack != 2)
        {
          util.yield();
          return;
        }
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    copter_change_x_by (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                this.sim_x = this.sim_x + Number(args.DISTANCE_DELTA);
                this._sim_target_x = this.sim_x;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this._sim_target_x, this.sim_y, this.sim_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          this.init_start_coordinates();
          this.x = this.x + Number(args.DISTANCE_DELTA);
          this.yielded_time_start = Date.now();
          this.yielded_time_now = Date.now();
          this.SendCordInterval =  setInterval(() =>{
            if (this.runtime.QCA.isQuadcopterConnected()){
                  this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
            }else{
                  clearInterval(this.SendCordInterval);
                  this.fack=0;
            }
          },100);
          this.fack = 1;
          util.yield();
          return;
        }
        else if(this.fack != 2)
        {
          if ((this.yielded_time_now - this.yielded_time_start ) >= this.yielded_max_time){
            this.fack=2;
          }
          this.nowx = this.runtime.QCA.get_coord("X");
          if(Math.abs(this.nowx-this.x)<this.delta)
          {
            this.fack=2;
          }
          this.yielded_time_now = Date.now();
          util.yield();
          return;
        }
        clearInterval(this.SendCordInterval);
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    copter_change_y_by (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                this.sim_y = this.sim_y - Number(args.DISTANCE_DELTA);
                this._sim_target_y = this.sim_y;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this.sim_x, this._sim_target_y, this.sim_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          this.init_start_coordinates();
          this.y = this.y - Number(args.DISTANCE_DELTA);
          this.yielded_time_start = Date.now();
          this.yielded_time_now = Date.now();
          this.SendCordInterval =  setInterval(() =>{
            if (this.runtime.QCA.isQuadcopterConnected()){
                  this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
            }else{
                  clearInterval(this.SendCordInterval);
                  this.fack=0;
            }
          },100);
          this.fack = 1;
          util.yield();
          return;
        }
        else if(this.fack != 2)
        {
          if ((this.yielded_time_now - this.yielded_time_start ) >= this.yielded_max_time){
            this.fack=2;
          }
          this.nowy = this.runtime.QCA.get_coord("Y");
          if(Math.abs(this.nowy-this.y)<this.delta)
          this.fack=2;
          this.yielded_time_now = Date.now();
          util.yield();
          return;
        }
        clearInterval(this.SendCordInterval);
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    copter_change_z_by (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                this.sim_z = this.sim_z + Number(args.DISTANCE_DELTA);
                if (this.sim_z < 0) this.sim_z = 0;
                this._sim_target_z = this.sim_z;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this.sim_x, this.sim_y, this._sim_target_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          this.init_start_coordinates();
          this.z = this.z + Number(args.DISTANCE_DELTA);
          console.log(`copter_change_z_by: ${this.z}`)
          this.SendCordInterval =  setInterval(() =>{
            if (this.runtime.QCA.isQuadcopterConnected()){
                  this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
            }else{
                  clearInterval(this.SendCordInterval);
                  this.fack=0;
            }
          },100);
          this.yielded_time_start = Date.now();
          this.yielded_time_now = Date.now();
          this.fack = 1;
          util.yield();
          return;
        }
        else if(this.fack != 2)
        {
          if ((this.yielded_time_now - this.yielded_time_start ) >= this.yielded_max_time){
            this.fack=2;
          }
          this.nowz = this.runtime.QCA.get_coord("Z");
          if(Math.abs(this.nowz-this.z)<this.delta)
          this.fack=2;
          this.yielded_time_now = Date.now();
          util.yield();
          return;
        }
        clearInterval(this.SendCordInterval);
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    copter_x_coord (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_x.toFixed(3));
        return Number(this.runtime.QCA.telemetry_palette_get_coord("X"));
    }

    copter_y_coord (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_y.toFixed(3));
        return Number(this.runtime.QCA.telemetry_palette_get_coord("Y"));
    }

    copter_yaw (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_yaw.toFixed(1));
        return Number(this.runtime.QCA.telemetry_palette_get_coord("W"));
    }

    copter_z_coord (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_z.toFixed(3));
        return Number(this.runtime.QCA.telemetry_palette_get_coord("Z"));
    }

    copter_fly_for_seconds_to_coords (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                const seconds = Number(args.SECONDS);
                const tx = Number(args.X_COORD);
                const ty = Number(args.Y_COORD) * -1;
                const tz = Number(args.Z_COORD);
                const vx = seconds > 0 ? (tx - this.sim_x) / seconds : 0;
                const vy = seconds > 0 ? (ty - this.sim_y) / seconds : 0;
                const vz = seconds > 0 ? (tz - this.sim_z) / seconds : 0;
                this._simClearInterval();
                this.sim_interval = setInterval(() => {
                    this.sim_x += vx * (SIM_STEP_MS / 1000);
                    this.sim_y += vy * (SIM_STEP_MS / 1000);
                    this.sim_z += vz * (SIM_STEP_MS / 1000);
                    this._simApplyState();
                }, SIM_STEP_MS);
                this.fack = 1;
                setTimeout(() => { this.fack = 2; }, seconds * 1000);
                util.yield();
                return;
            } else if (this.fack !== 2) {
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          this.init_start_coordinates();
          let vx = (Number(args.X_COORD)-this.x)/Number(args.SECONDS);
          let vy = (Number(args.Y_COORD)-this.y)/Number(args.SECONDS);
          console.log(`CHLENvx!: ${vx}`)
          console.log(`CHLENvy!: ${vy}`)
          this.z = Number(args.Z_COORD);
          this.runtime.QCA.move_with_speed(vx,vy,0,this.z);
          this.fack=1;
          let time_to_fly = Number(args.SECONDS)*1000;
          setTimeout(() => { this.fack = 2; }, time_to_fly);
          util.yield();
          return;
        }
        else if ( this.fack != 2)
        {
          util.yield();
          return;
        }
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    copter_fly_to_coords (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                this._sim_target_x = Number(args.X_COORD);
                this._sim_target_y = Number(args.Y_COORD) * -1;
                this._sim_target_z = Number(args.Z_COORD);
                this.sim_x = this._sim_target_x;
                this.sim_y = this._sim_target_y;
                this.sim_z = this._sim_target_z;
                this._simStartMoveToCoord(this._sim_target_x, this._sim_target_y, this._sim_target_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this._sim_target_x, this._sim_target_y, this._sim_target_z, this.sim_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        if(this.fack==0)
        {
          this.x_telemetry_delta =  this.runtime.QCA.get_x_telemetry_delta();
          this.y_telemetry_delta =  this.runtime.QCA.get_y_telemetry_delta();
          this.init_start_coordinates();
          this.x = Number(args.X_COORD);
          this.y = Number(args.Y_COORD) * -1;
          this.z = Number(args.Z_COORD);
          if (this.x > 0){
                this.x = this.x + this.x_telemetry_delta;
              }else{
                this.x = this.x + this.x_telemetry_delta;
              }
          if (this.y > 0){
                this.y = this.y + this.y_telemetry_delta;
            }else{
                this.y = this.y + this.y_telemetry_delta;
            }
          this.yielded_time_start = Date.now();
          this.yielded_time_now = Date.now();
          this.SendCordInterval =  setInterval(() =>{
            if (this.runtime.QCA.isQuadcopterConnected()){
                  this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
            }else{
                  clearInterval(this.SendCordInterval);
                  this.fack=0;
            }
          },100);
          this.fack=1;
          util.yield();
          return;
        }
        else if(this.fack != 2)
        {
          if ((this.yielded_time_now - this.yielded_time_start ) >= this.yielded_max_time){
            this.fack=2;
          }
          this.nowx =Number( this.runtime.QCA.get_coord("X"));
          this.nowy =Number( this.runtime.QCA.get_coord("Y"));
          this.nowz =Number( this.runtime.QCA.get_coord("Z"));
          if((Math.abs(this.nowx-this.x)<this.delta)&&(Math.abs(this.nowy-this.y)<this.delta)&&(Math.abs(this.nowz-this.z)<this.delta))
          this.fack=2;
          this.yielded_time_start = Date.now();
          util.yield();
          return;
        }
        clearInterval(this.SendCordInterval);
        this.runtime.QCA.move_with_speed(0,0,0,this.z);
        this.fack=0;
    }

    cast_yaw_to_360 (yaw) {
        return this._castYawTo360(yaw);
    }

    copter_rotate (args, util) {
        if (this.runtime.sim_copter_ac) {
            if (this.fack === 0) {
                this.sim_yaw += Number(args.DEGREES);
                if ((this.sim_yaw > 360) || (this.sim_yaw < -360)) {
                    this.sim_yaw = this._castYawTo360(this.sim_yaw);
                }
                this._sim_target_yaw = this.sim_yaw;
                this._simStartMoveToCoord(this.sim_x, this.sim_y, this.sim_z, this.sim_yaw);
                this.yielded_time_start = Date.now();
                this.fack = 1;
                util.yield();
                return;
            } else if (this.fack !== 2) {
                if ((Date.now() - this.yielded_time_start) >= this.yielded_max_time) this.fack = 2;
                if (this._simIsAtTarget(this.sim_x, this.sim_y, this.sim_z, this._sim_target_yaw)) this.fack = 2;
                util.yield();
                return;
            }
            this._simClearInterval();
            this._simApplyState();
            this.fack = 0;
            return;
        }

        // --- Hardware path ---
        {
          if(this.fack==0)
          {
            this.init_start_coordinates();
            this.yaw += Number(args.DEGREES);
            if ((this.yaw > 360) || (this.yaw < -360)){
              this.yaw = this.cast_yaw_to_360(this.yaw);
            }
            this.yielded_time_start = Date.now();
            this.yielded_time_now = Date.now();
            this.SendCordInterval =  setInterval(() =>{
              if (this.runtime.QCA.isQuadcopterConnected()){
                    this.runtime.QCA.move_to_coord(this.x,this.y,this.z,this.yaw);
              }else{
                    clearInterval(this.SendCordInterval);
                    this.fack=0;
              }
            },100);
            this.fack=1;
            util.yield();
            return;
          }
          else if(this.fack != 2)
          {
            if ((this.yielded_time_now - this.yielded_time_start ) >= this.yielded_max_time){
              this.fack=2;
            }
            this.noww = Number(this.runtime.QCA.get_coord("W"));
            if(Math.abs(this.noww-this.yaw)<3)
            this.fack=2;
            this.yielded_time_now = Date.now();
            util.yield();
            return;
          }
          clearInterval(this.SendCordInterval);
          this.runtime.QCA.move_with_speed(0,0,0,this.z);
          this.fack=0;
        }
    }

    copter_set_direction (args, util) {
        switch (args.DIRECTION) {
          case 'direction_forward':
            this.dir = 0;
          break;
          case 'direction_right':
            this.dir = 270;
          break;
          case 'direction_backward':
            this.dir = 180;
          break;
          case 'direction_left':
            this.dir = 90;
          break;
        }
    }

    copter_direction (args, util) {
        if (this.runtime.sim_copter_ac) return Number(this.sim_yaw.toFixed(1));
        return this.runtime.QCA.get_coord("W");
    }

    init_start_coordinates () {
        this.yaw = Number(this.runtime.QCA.get_coord("W"));
        this.x = Number(this.runtime.QCA.get_coord("X"));
        this.y = Number(this.runtime.QCA.get_coord("Y"));
        this.z = Number(this.runtime.QCA.get_coord("Z"));
    }
}

module.exports = Scratch3QuadcopterBlocks;
