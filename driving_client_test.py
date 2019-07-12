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
        
        car_controls.steering = self.change_heading(sensing_info)
        
        if self.count < 50:
            car_controls.steering = 1
            car_controls.throttle = 0.5
        elif self.check_road_limit(sensing_info):
            if sensing_info.to_middle > 0:
                car_controls.steering = car_controls.steering - 0.3
            else:
                car_controls.steering = car_controls.steering + 0.3

        self.count += 1
        print("Processing Count: {}".format(self.count))
        
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
