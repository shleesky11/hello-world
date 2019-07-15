from drive_controller import DrivingController
import math


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.is_debug = True
        self.collision_flag = True
        self.st_collided = 0
        self.prev_vel = 0.0
        self.prev_count = 0
        self.prev_throttle = 0.0
        self.count = 0

        #Timer
        self.timer = {}
        self.set_timer(100,10)

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        
        if self.is_debug:
            print("=========================================================")
            print("to middle: {}".format(sensing_info.to_middle))

            print("collided: {}".format(sensing_info.collided))
            print("car speed: {} km/h".format(sensing_info.speed))

            print("is moving forward: {}".format(sensing_info.moving_forward))
            print("moving angle: {}".format(sensing_info.moving_angle))
            print("lap_progress: {}".format(sensing_info.lap_progress))

            print("track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            
            print("Current Collied Status: {}".format(self.st_collided))
            print("=========================================================")

        ###########################################################################

        # Moving straight forward
        car_controls.steering = 0
        car_controls.throttle = 1
        car_controls.brake = 0

        #car driving steering
        car_controls.steering = self.change_heading(sensing_info)
        cp_curving = 0
        cp_obstacle = False

        timer_list = self.get_timer()
        for id in timer_list:
            if id == 100:
                pass
        
        angle_diff = []
        for i in range(1, len(sensing_info.track_forward_angles)):
            diff = sensing_info.track_forward_angles[i] - sensing_info.track_forward_angles[i-1]
            angle_diff.append(diff)
            if cp_curving == 0 and abs(diff) > 5:
                if diff > 0:
                    cp_curving = 1
                else:
                    cp_curving = 2
        if len(sensing_info.track_forward_obstacles) > 0:
            cp_obstacle = True
        road_status = self.road_status(sensing_info, cp_obstacle, cp_curving)
        print("road_status: {}".format(road_status))
 
        if sensing_info.speed > 0 and self.st_collided == 0:
            #road_status: 0 => no obstacle and strait => pass
            
            #road_status: 1 => right curve and no obstacle
            if road_status == 1:
                if sensing_info.speed > 70:
                    car_controls.brake = 0.2
            
            #road_status: 2 => left curve and no obstacle
            elif road_status == 2:
                 if sensing_info.speed > 70:
                    car_controls.brake = 0.2

            #road_status: 3 => strait and obstacle
            elif road_status == 3:
                ob_to_middle = sensing_info.track_forward_obstacles[0].get('to_middle')
                ob_dist = sensing_info.track_forward_obstacles[0].get('dist')
                if abs(ob_to_middle - sensing_info.to_middle) < 3:
                    if ob_to_middle < 0:
                        car_controls.steering += math.tan(abs(ob_to_middle)/ob_dist)
                        print("steering +: {}".format(math.tan(abs(ob_to_middle)/ob_dist)))
                    else:
                        car_controls.steering -= math.tan(abs(ob_to_middle)/ob_dist)
                        print("steering -: {}".format(math.tan(abs(ob_to_middle)/ob_dist)))
                    if ob_dist < 20:
                        self.emergency_process(car_controls, sensing_info)

            #road_status: 4 => right curve and obstacle
            elif road_status == 4:
                if sensing_info.speed > 70:
                    car_controls.brake = 0.2
                
                ob_to_middle = sensing_info.track_forward_obstacles[0].get('to_middle')
                ob_dist = sensing_info.track_forward_obstacles[0].get('dist')
                if abs(ob_to_middle - sensing_info.to_middle) < 3:
                    if ob_to_middle < 0:
                        car_controls.steering += math.tan(abs(ob_to_middle)/ob_dist)
                        print("steering +: {}".format(math.tan(abs(ob_to_middle)/ob_dist)))
                    else:
                        car_controls.steering -= math.tan(abs(ob_to_middle)/ob_dist)
                        print("steering -: {}".format(math.tan(abs(ob_to_middle)/ob_dist)))
                    if ob_dist < 20:
                        self.emergency_process(car_controls, sensing_info)

            #road_status: 5 => left curve and obstalce
            elif road_status == 5:
                if sensing_info.speed > 70:
                    car_controls.brake = 0.3
                
                ob_to_middle = sensing_info.track_forward_obstacles[0].get('to_middle')
                ob_dist = sensing_info.track_forward_obstacles[0].get('dist')
                if abs(ob_to_middle - sensing_info.to_middle) < 5:
                    if ob_to_middle <= 0:
                        car_controls.steering += math.tan(math.radians(-1*(ob_to_middle-3)/ob_dist))
                    else:
                        car_controls.steering -= math.tan(math.radians((ob_to_middle+3)/ob_dist))

            if self.check_road_limit(sensing_info):
                if sensing_info.to_middle > 0:
                    if sensing_info.speed < 30:
                        car_controls.steering = car_controls.steering - 0.5
                    elif sensing_info.speed < 80:
                        car_controls.steering = car_controls.steering - 0.2
                    else:
                        car_controls.steering = car_controls.steering - 0.1
                else:
                    if sensing_info.speed < 30:
                        car_controls.steering = car_controls.steering + 0.5
                    elif sensing_info.speed < 80:
                        car_controls.steering = car_controls.steering + 0.2
                    else:
                        car_controls.steering = car_controls.steering + 0.1

        #collision occurrance
        if self.st_collided == 0 and sensing_info.collided:
            #car is located at side area
            if abs(sensing_info.to_middle) > self.half_road_limit:
                self.st_collided = 2
            #car is located in road
            else:
                self.st_collided = 1
        
        if self.st_collided != 0:
            self.set_brake(car_controls, sensing_info, 0.0)
            #processing for car collision at  road area
            if self.st_collided == 1:
                if len(sensing_info.track_forward_obstacles) > 0 and sensing_info.track_forward_obstacles[0].get('dist') > 0:
                    mid_from_obstacle = abs(sensing_info.track_forward_obstacles[0].get('to_middle') - sensing_info.to_middle)
                    dist_from_obstacle = mid_from_obstacle + sensing_info.track_forward_obstacles[0].get('dist')

                    print("dist_from_obstacle: {}".format(dist_from_obstacle))
                    if dist_from_obstacle < 10:
                        if self.prev_count != 0 and (self.count - self.prev_count) % 5 == 0:
                            if abs(sensing_info.speed - self.prev_vel) < 1:
                                self.prev_throttle = -1*self.prev_throttle
                            self.prev_count = self.count
                            self.prev_vel = sensing_info.speed
                        if self.prev_count == 0:
                            self.prev_count = self.count
                            self.prev_vel = sensing_info.speed
                            self.prev_throttle = -1
                        car_controls.throttle = self.prev_throttle
                        if sensing_info.speed < 20 and sensing_info.moving_angle > 30:
                            car_controls.steering = car_controls.steering*0.5
                    else:
                        self.st_collided = 0
                        self.reset_prev_val()
                        print("Collistion Resolved_01")
                else:
                    self.st_collided = 0
                    self.reset_prev_val()
                    print("Collistion Resolved_02")

            #processing for car collision outside road
            elif self.st_collided == 2:
                if abs(sensing_info.to_middle) > self.half_road_limit:
                    if self.prev_count != 0 and (self.count - self.prev_count) % 5 == 0:
                        if abs(sensing_info.speed - self.prev_vel) < 0.3:
                            self.prev_throttle = -1*self.prev_throttle
                        self.prev_count = self.count
                        self.prev_vel = sensing_info.speed
                    if self.prev_count == 0:
                        self.prev_count = self.count
                        self.prev_vel = sensing_info.speed
                        self.prev_throttle = -1
                    car_controls.throttle = self.prev_throttle
                    
                    if abs(sensing_info.moving_angle) > 45:
                        if sensing_info.speed < 10:
                            if sensing_info.to_middle < 0:
                                car_controls.steering = -0.3
                            else:
                                car_controls.steering = 0.3
                        else:
                            if sensing_info.to_middle < 0:
                                car_controls.steering = -0.5
                            else:
                                car_controls.steering = 0.5
                    else:
                        if sensing_info.speed < 10:
                            if sensing_info.to_middle < 0:
                                car_controls.steering = 0.3
                            else:
                                car_controls.steering = -0.3
                        else:
                            if sensing_info.to_middle < 0:
                                car_controls.steering = 0.5
                            else:
                                car_controls.steering = -0.5
                else:
                    self.st_collided = 0
                    self.reset_prev_val()
                    print("Collistion Resolved_03")

            print("Count: {}, Prov_Count: {}, Prev_throttle: {}, Prev_vel: {}".format(self.count, self.prev_count, self.prev_throttle, self.prev_vel))

        print("Current Collied Status: {}".format(self.st_collided))
        
        if self.st_collided == 0 and not sensing_info.moving_forward:
            if sensing_info.to_middle < 0:
                car_controls.steering = -1
            else:
                car_controls.steering = 1

        self.count += 1
        if self.is_debug:
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls


    #Define Custom Functions
    def check_road_limit(self, sensing_info):
        return self.half_road_limit/3 < abs(sensing_info.to_middle)
    
    def change_heading(self, sensing_info):
        if sensing_info.speed > 0:
            ret_heading = -1*math.sin(math.radians(sensing_info.moving_angle))
        else:
            ret_heading = math.sin(math.radians(sensing_info.moving_angle))
        return ret_heading

    def reset_prev_val(self):
        self.prev_count = 0
        self.prev_vel = 0.0
        self.prev_throttle = 0
        
    def set_brake(self, car_controls, sensing_info, value):
        if sensing_info.speed > 70:
            car_controls.brake = value
        elif sensing_info.speed > 50:
            car_controls.brake = value * 0.5

    
    def set_timer(self, id, interval):
        if not id in self.timer:
            self.timer[id] = [interval, self.count]
            return True
        else:
            return False
    
    def free_timer(self, id):
        if id in self.timer:
            del self.timer[id]

    def get_timer(self):
        ret_keys = []
        for key in list(self.timer.keys()):
            if (self.count - self.timer[key][1]) % self.timer[key][0] == 0:
                ret_keys.append(key)
        return ret_keys
    
    def calc_speed(self, sensing_info):
        return sensing_info.speed*1000/3600

    def road_status(self, sensing_info, cp_obstacle, cp_curving):
        ret_status = 0 #strait and no obstacle
        if not cp_obstacle:
            if cp_curving == 1:
                ret_status = 1 #right curve and no obstacle
            elif cp_curving == 2:
                ret_status = 2 #left curve and no obstacle
        else:
            if cp_curving == 0:
                ret_status = 3 #strait and obstacle
                #straight and left obstacle
            elif cp_curving == 1:
                ret_status = 4 #right curve and obstacle
            elif cp_curving == 2:
                ret_status = 5 #left curve and obstalce
        return ret_status

    def emergency_process(self, car_controls, sensing_info):
        print("Emergency_Process is excuted")
        ob_to_middle = sensing_info.track_forward_obstacles[0].get('to_middle')
        
        if sensing_info.speed > 90:
            car_controls.brake = 1.0
            car_controls.throttle = 0.6
        elif sensing_info.speed > 70:
            car_controls.brake = 1.0
            car_controls.throttle = 0.7
        elif sensing_info.speed > 50:
            car_controls.brake = 0.5

        if abs(ob_to_middle) - abs(sensing_info.to_middle) > 0:
            if ob_to_middle < 0:
                car_controls.steering += 0.3
                print("steering: +0.3")
            else:
                car_controls.steering -= 0.3
                print("steering: -0.3")
        else:
            if sensing_info.to_middle < 0:
                car_controls.steering -= 0.3
                print("steering: -0.3")
            else:
                car_controls.steering += 0.3
                print("steering: +0.3")


    
    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name
 

if __name__ == '__main__':
    client = DrivingClient()
    client.run()
