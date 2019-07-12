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
        self.count = 0

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
        
        #차량 주행 핸들링
        car_controls.steering = self.change_heading(sensing_info)
        if self.check_road_limit(sensing_info):
            if sensing_info.to_middle > 0:
                car_controls.steering = car_controls.steering - 0.2
            else:
                car_controls.steering = car_controls.steering + 0.2
        
        #장애물 회피
        if len(sensing_info.track_forward_obstacles) > 0:
            diff_with_ob = abs(sensing_info.to_middle - sensing_info.track_forward_obstacles[0].get('to_middle'))
            dist_from_ob = sensing_info.track_forward_obstacles[0].get('dist')
            if dist_from_ob < 50 and diff_with_ob < 2:
                if sensing_info.speed > 50:
                    car_controls.brake = 0.2
                if sensing_info.track_forward_obstacles[0].get('to_middle') > 0:
                    car_controls.steering = car_controls.steering - 0.2
                else:
                    car_controls.steering = car_controls.steering + 0.2
                    
        #급커브 기동


        #충돌이 발생
        if self.st_collided == 0 and sensing_info.collided:
            if len(sensing_info.track_forward_obstacles) == 0:
                self.st_collided = 2
            elif sensing_info.track_forward_obstacles[0].get('dist') > 10:
                self.st_collided = 2
            elif abs(sensing_info.to_middle) > 10:
                self.st_collided = 2
            else:
                self.st_collided = 1
        
        
        if self.st_collided != 0:
            car_controls.brake = 0
            #차량이 도로 장애물에 충돌
            if self.st_collided == 1:
                #차량이 전진 중에 충돌 발생 => 후진 수행
                if sensing_info.moving_forward:
                    if sensing_info.collided:
                        car_controls.throttle = -1
                        if abs(car_controls.steering) > 1:
                            if car_controls.steering > 0:
                                car_controls.steering = 1
                            else:
                                car_controls.steering = -1
                        else:
                            car_controls.steering = -1*car_controls.steering
                    else:
                        car_controls.throttle = 1
                else:
                    #차량이 후진 중에 충돌 발생 => 전진 수행
                    if sensing_info.collided:
                        car_controls.throttle = 1
                    elif sensing_info.track_forward_obstacles[0].get('dist') < 5:
                        car_controls.throttle = -1
                        car_controls.steering = -1*car_controls.steering
                    else:
                        car_controls.throttle = 1
                
                #도로 중간 장애물 충돌 상황 제거
                if len(sensing_info.track_forward_obstacles) == 0 or sensing_info.track_forward_obstacles[0].get('dist') > 5:
                    self.st_collided = 0
            
            #차량이 사이드 펜스에 충돌
            elif self.st_collided == 2:
                if sensing_info.to_middle < 0:
                    car_controls.throttle = -1
                    car_controls.steering = -0.8
                else:
                    car_controls.throttle = -1
                    car_controls.steering = 0.8
 
                if abs(sensing_info.to_middle) < 8:
                    self.st_collided = 0

                if sensing_info.collided and not sensing_info.moving_forward:
                    car_controls.throttle = 1
                    
         
        print("Current Collied Status: {}".format(self.st_collided))
        
        if self.st_collided == 0 and not sensing_info.moving_forward:
            if sensing_info.to_middle > 0:
                car_controls.steering = -1
            else:
                car_controls.steering = 1

        
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
       return -1*math.sin(math.radians(sensing_info.moving_angle))
    
    #def change_path_line(self):
        

    
    
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
