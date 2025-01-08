
import numpy as np
import math




class Trajectory:
    print(999)
    step_counter = 0
    func_num = 2
    obstacles_coords = [] 
    
    def __init__(self, plot):
        self.plot = plot
        
    def calculate_step(self):
        if self.step_counter == 0:
            self.calculate_distances_and_angles()
            self.plot.create_distances_and_angles()
        elif self.step_counter == 1:
            self.calculate_detour_point()
            self.plot.create_detour_point()
            
        self.step_counter += 1
        if self.step_counter == self.func_num:
            self.step_counter = 0
            
        
            
            
    def calculate_distances_and_angles(self): #расчёт расстояний до точек препятствия и углов до них
        self.obstacles_distance = [] #расстояния до точек препятствия
        self.obstacles_angle = [] #углы до точек препятствия
        self.target_distance, self.target_angle = self.distance_and_angle(self.plot.robot_coords[0], self.plot.robot_coords[1], 
                                                                          self.plot.target_coords[0], self.plot.target_coords[1]) #угол и расстояние до цели
        
        for obstacle in self.plot.obstacles_coords:
            for i in range(int(len(obstacle)/2)):
                x = obstacle[i*2]
                y = obstacle[i*2+1]
                distance, angle = self.distance_and_angle(self.plot.robot_coords[0], self.plot.robot_coords[1], 
                                                          x, y)
                self.obstacles_coords.append([x, y])
                self.obstacles_distance.append(distance)
                self.obstacles_angle.append(angle)

  #2 cnhjrf      
    def calculate_detour_point(self): 
        self.angles_list = []

        for i in range(len(self.obstacles_angle)):
            if i > 0 and (abs(self.obstacles_angle[i] - self.angles_list[-1]) > np.pi):
                    if self.angles_list[-1] < 0:
                        angle = self.obstacles_angle[i] - np.pi*2
                    else:
                        angle = self.obstacles_angle[i] + np.pi*2
            else:
                angle = self.obstacles_angle[i]
            self.angles_list.append(angle)

        min_1 = min(self.obstacles_angle)
        max_1 = max(self.obstacles_angle)
        min_2 = min(self.angles_list)
        max_2 = max(self.angles_list)
        print(self.obstacles_angle, min_1, max_1)
        print(self.angles_list, min_2, max_2)

#-------------------------------------------------------------------------

        self.point_track_list = []
        track_list_one = []
        self.point_obstacle_list = []
        finish = False

        new_angle = self.angles_list
        new_distance = self.obstacles_distance
        znak_list = []
        plus_count = 0
        minus_count = 0
        TA = self.target_angle
        RP = self.plot.robot_coords

        #определяем знаки у точек, относительно целевой
        self.angles_list = []
      
        for i in range(len(new_angle)):
            if i > 0 and (abs(new_angle[i] - self.angles_list[-1]) > np.pi):
                if self.angles_list[-1] < 0:
                    angle = new_angle[i] - np.pi*2
                else:
                    angle = new_angle[i] + np.pi*2
            else:
                angle = new_angle[i]
            self.angles_list.append(angle)

            if (abs(self.angles_list[i] - TA) > np.pi):
                if self.angles_list[i] < 0:
                    TA = TA - np.pi*2
                else:
                    TA = TA + np.pi*2

            znak = self.angles_list[i] - TA
            if znak < 0:
                minus_count += 1
            else:
                plus_count += 1
            znak_list.append(znak)


        #определяем угол в зависимости от знаков
        #если углов нет - смотреть ближайший к нам и объехать его (если надо)
        if plus_count == 0 or minus_count == 0:
            print("прямая")
            I = 0
            pluss_count = 10
            for i in range(len(znak_list)):
                if pluss_count > abs(znak_list[i]):
                    I = i
                    pluss_count = abs(znak_list[i])

            x_p1,y_p1,x_p2,y_p2 = self.point_to_round(RP[0],RP[1],self.obstacles_coords[I][0],self.obstacles_coords[I][1],0.3)

            buffer_point = []
            new_angle = []
            znak_list = []
            plus_count = 0
            minus_count = 0
            i_min = -1
            b_m = 1000

            #определяем знаки для нового набора точек, относительно целевой
            buffer_point.append([x_p1,y_p1])
            buffer_point.append([x_p2,y_p2])
            for i in range(2):
                nd, na = self.distance_and_angle(RP[0],RP[1],buffer_point[i][0],buffer_point[i][1])
                new_angle.append(na)
            
            self.angles_list = []

            for i in range(len(new_angle)):
                if i > 0 and (abs(new_angle[i] - self.angles_list[-1]) > np.pi):
                    if self.angles_list[-1] < 0:
                        angle = new_angle[i] - np.pi*2
                    else:
                        angle = new_angle[i] + np.pi*2
                else:
                    angle = new_angle[i]
                self.angles_list.append(angle)

                if (abs(self.angles_list[i] - TA) > np.pi):
                    if self.angles_list[i] < 0:
                        TA = TA - np.pi*2
                    else:
                        TA = TA + np.pi*2

                znak = self.angles_list[i] - TA
                if znak < 0:
                    minus_count += 1
                else:
                    plus_count += 1
                znak_list.append(znak)
                if abs(znak_list[i]) < b_m:
                    i_r = i
                    b_m = abs(znak_list[i])

            if plus_count == 0 or minus_count == 0:
                self.point_track_list.append([[RP[0], RP[1]], [self.plot.target_coords[0], self.plot.target_coords[1]], [-1000,-1000]])
                track_list_one = []
                finish = True
            else:
                self.point_obstacle_list.append(self.obstacles_coords[I])
                self.point_track_list.append([RP, buffer_point[i_r], self.obstacles_coords[I]])

                x_p1, y_p1, x_p2, y_p2 = self.point_to_round(self.plot.target_coords[0], self.plot.target_coords[1],self.obstacles_coords[I][0],self.obstacles_coords[I][1],0.3)

                buffer_point = []
                new_angle = []
                znak_list = []
                plus_count = 10
                minus_count = 0
                TD, TA = self.distance_and_angle(self.plot.target_coords[0],self.plot.target_coords[1],self.plot.robot_coords[0],self.plot.robot_coords[1])
                RP = self.plot.robot_coords

                #определяем знаки для нового набора точек, относительно целевой
                buffer_point.append([x_p1,y_p1])
                buffer_point.append([x_p2,y_p2])
                for i in range(2):
                    nd, na = self.distance_and_angle(self.plot.target_coords[0],self.plot.target_coords[1],buffer_point[i][0],buffer_point[i][1])
                    new_angle.append(na)
            
                if (abs(new_angle[0] - TA) > np.pi):
                    if new_angle[0] < 0:
                        TA = TA - np.pi*2
                    else:
                        TA = TA + np.pi*2
      
                for i in range(len(new_angle)):
                    znak = new_angle[i] - TA
                    znak_list.append(new_angle[i] - TA)
                    if abs(znak_list[i]) < plus_count:
                        plus_count = abs(znak_list[i])
                        minus_count = i

                #определяем нужную точку касания и заносим её в журнал
                if minus_count == 0:
                    self.point_track_list.append([[x_p1, y_p1], self.plot.target_coords, [-1000,-1000]])
                else:
                    self.point_track_list.append([[x_p2, y_p2], self.plot.target_coords, [-1000,-1000]])
                track_list_one = []
                finish = True

        #если угол один - объехать его
        elif plus_count == 1 or minus_count == 1:
            print("угол")
            #определяем угол
            I = 0
            for i in range(len(znak_list)):
                if plus_count == 1:
                    if znak_list[i]>0:
                        I = i
                        break
                else:
                    if znak_list[i]<0:
                        I = i
                        break

            self.point_obstacle_list.append(self.obstacles_coords[I])

            #определяем две точки касания
            x_p1, y_p1, x_p2, y_p2 = self.point_to_round(RP[0],RP[1],self.obstacles_coords[I][0],self.obstacles_coords[I][1],0.3)
            
            buffer_point = []
            new_angle = []
            znak_list = []
            plus_count = 0
            minus_count = 0
            TA = self.target_angle
            RP = self.plot.robot_coords

            #определяем знаки для нового набора точек, относительно целевой
            buffer_point.append([x_p1,y_p1])
            buffer_point.append([self.obstacles_coords[I][0],self.obstacles_coords[I][1]])
            buffer_point.append([x_p2,y_p2])
            for i in range(3):
                nd, na = self.distance_and_angle(RP[0],RP[1],buffer_point[i][0],buffer_point[i][1])
                new_angle.append(na)
      
            if (abs(new_angle[0] - TA) > np.pi):
                if new_angle[0] < 0:
                    TA = TA - np.pi*2
                else:
                    TA = TA + np.pi*2

            for i in range(len(new_angle)):
                znak = new_angle[i] - TA
                znak_list.append(new_angle[i] - TA)
                if abs(znak_list[i]) > plus_count:
                    plus_count = abs(znak_list[i])
                    minus_count = i

            #определяем нужную точку касания и заносим её в журнал
            if minus_count == 0:
                self.point_track_list.append([RP, [x_p1, y_p1], self.obstacles_coords[I]])
            else:
                self.point_track_list.append([RP, [x_p2, y_p2], self.obstacles_coords[I]])
            #point_track_list.append(track_list_one)
            track_list_one = []

            #строим касательную от цели до угла
            x_p1, y_p1, x_p2, y_p2 = self.point_to_round(self.plot.target_coords[0], self.plot.target_coords[1],self.obstacles_coords[I][0],self.obstacles_coords[I][1],0.3)

            buffer_point = []
            new_angle = []
            znak_list = []
            plus_count = 0
            minus_count = 0
            TD, TA = self.distance_and_angle(self.plot.target_coords[0],self.plot.target_coords[1],self.plot.robot_coords[0],self.plot.robot_coords[1])
            RP = self.plot.robot_coords

            #определяем знаки для нового набора точек, относительно целевой
            buffer_point.append([x_p1,y_p1])
            buffer_point.append([self.obstacles_coords[I][0],self.obstacles_coords[I][1]])
            buffer_point.append([x_p2,y_p2])
            for i in range(3):
                nd, na = self.distance_and_angle(self.plot.target_coords[0],self.plot.target_coords[1],buffer_point[i][0],buffer_point[i][1])
                new_angle.append(na)
            
            if (abs(new_angle[0] - TA) > np.pi):
                if new_angle[0] < 0:
                    TA = TA - np.pi*2
                else:
                    TA = TA + np.pi*2

            for i in range(len(new_angle)):
                znak = new_angle[i] - TA
                znak_list.append(new_angle[i] - TA)
                if abs(znak_list[i]) > plus_count:
                    plus_count = abs(znak_list[i])
                    minus_count = i

            #определяем нужную точку касания и заносим её в журнал
            if minus_count == 0:
                self.point_track_list.append([[x_p1, y_p1], self.plot.target_coords, [-1000,-1000]])
            else:
                self.point_track_list.append([[x_p2, y_p2], self.plot.target_coords, [-1000,-1000]])
 #           point_track_list.append(track_list_one)
            track_list_one = []
            finish = True



        #если углов два - объехать ближайшие к нам
        else:
            print("два угла")
            #определяем ближайший к прямой (робот - цель ) угол
            I0 = 0
            I1 = 0
            min_ug = 90
            znak = 1
            for i in range(len(znak_list)):
                if abs(znak_list[i]) < abs(min_ug):
                    I = i
                    min_ug = znak_list[i]
                    if znak_list[i] > 0:
                        znak = 1
                    else:
                        znak = -1

            #определяем по туже сторону от прямой, что и найденный на предыдущем шаге
            for i in range (len(znak_list)):
                if min_ug * znak_list[i] >= 0 and min_ug != znak_list[i]:
                    I1 = i
                    break

            #формуируем список точек для объезда
            if new_distance[I0] < new_distance[I1]:
                if new_angle[I0] < new_angle[I1]:
                    self.point_obstacle_list.append(self.obstacles_coords[I1])
                else:
                    self.point_obstacle_list.append(self.obstacles_coords[I0])
                    self.point_obstacle_list.append(self.obstacles_coords[I1])
            else:
                if new_angle[I0] < new_angle[I1]:
                    self.point_obstacle_list.append(self.obstacles_coords[I1])
                    self.point_obstacle_list.append(self.obstacles_coords[I0])
                else:
                    self.point_obstacle_list.append(self.obstacles_coords[I0])






        print(self.point_track_list)
        print(self.point_obstacle_list)

#-------------------------------------------------------------
    def distance_and_angle(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        distance = round(np.sqrt(dx**2 + dy**2), 2)
        angle = round(math.atan2(dy, dx), 2)   
        return(distance, angle)

#-------------------------------------------------------------

    def discriminant(self, a, b, c):
        dis = (b**2) - (4*a*c)
        if dis >= 0:
            x1 = round(((-b + np.sqrt(dis)) / (2*a)), 2)
            x2 = round(((-b - np.sqrt(dis)) / (2*a)), 2)
            return (x1, x2)
        else:
            return None

    def calculate_new_disatance_and_angle(self, x_0, y_0):
        for obstacle in self.plot.obstacles_coords:
            for i in range(int(len(obstacle)/2)):
                x = obstacle[i*2]
                y = obstacle[i*2+1]
                distance, angle = self.distance_and_angle(x_0, y_0, 
                                                          x, y)
                self.obstacles_coords.append([x, y])
                self.obstacles_distance.append(distance)
                self.obstacles_angle.append(angle)

    def point_to_round(self, xp,yp,xr,yr,r):
        gip, ug = self.distance_and_angle(xp, yp, xr, yr)
        rad = np.sqrt(gip**2 - r**2)
        A = xr - xp
        B = xr + xp
        C = yr - yp
        D = yr + yp
        R = rad**2 - r**2
        O = (B/2) + ((C*D)/(A*2)) + (R/(A*2))
        a_1 = (C/A)**2 + 1
        b_1 = 2*(O - xp)*(C/A) + 2*yp
        c_1 = (O-xp)**2 + yp**2 - rad**2
        Y_1, Y_2 = self.discriminant(a_1, -1*b_1, c_1)
        X_1 = O - (C/A)*Y_1
        X_2 = O - (C/A)*Y_2
        return (X_1, Y_1, X_2, Y_2)