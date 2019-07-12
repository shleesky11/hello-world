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
        if self.check_road_limit(sensing_info):
            if sensing_info.to_middle > 0:
                car_controls.steering = car_controls.steering - 0.2
            else:
                car_controls.steering = car_controls.steering + 0.2

        #avoiding from obstacles
        if len(sensing_info.track_forward_obstacles) > 0:
            diff_with_ob = abs(sensing_info.to_middle - sensing_info.track_forward_obstacles[0].get('to_middle'))
            dist_from_ob = sensing_info.track_forward_obstacles[0].get('dist')
            if dist_from_ob < 50 and diff_with_ob < 2:
                if sensing_info.speed > 50:
                    car_controls.brake = 0.2
                '''
                if sensing_info.track_forward_obstacles[0].get('to_middle') > 0:
                    car_controls.steering = car_controls.steering - 0.2
                else:
                    car_controls.steering = car_controls.steering + 0.2
                '''
        #rapid curving handling


        #collision occurrance
        if self.st_collided == 0 and sensing_info.collided:
            #car is located at side area
            if abs(sensing_info.to_middle) > self.half_road_limit + 2:
                self.st_collided = 2
            #car is located in road
            else:
                self.st_collided = 1
        
        if self.st_collided != 0:
            car_controls.brake = 0
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
                        if abs(sensing_info.speed - self.prev_vel) < 1:
                            self.prev_throttle = -1*self.prev_throttle
                        self.prev_count = self.count
                        self.prev_vel = sensing_info.speed
                    if self.prev_count == 0:
                        self.prev_count = self.count
                        self.prev_vel = sensing_info.speed
                        self.prev_throttle = -1
                    car_controls.throttle = self.prev_throttle
                else:
                    self.st_collided = 0
                    self.reset_prev_val()
                    print("Collistion Resolved_03")

            print("Count: {}, Prov_Count: {}, Prev_throttle: {}, Prev_vel: {}".format(self.count, self.prev_count, self.prev_throttle, self.prev_vel))

        print("Current Collied Status: {}".format(self.st_collided))
        
        if self.st_collided == 0 and not sensing_info.moving_forward:
            if sensing_info.to_middle > 0:
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
        return -1*math.sin(math.radians(sensing_info.moving_angle))

    def reset_prev_val(self):
        self.prev_count = 0
        self.prev_vel = 0.0
        self.prev_throttle = 0

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
