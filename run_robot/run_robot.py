import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from matplotlib.patches import Rectangle
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider
from matplotlib.widgets import TextBox
from matplotlib.patches import Circle
from trk import Trajectory
import math


class Plot:
    
    button_width = 0.08
    button_height = 0.025
    entry_width = 0.02
    entry_height = 0.025
    color = 'green'
    hovercolor = 'yellow'
    zoom_factor = 1.1  # Коэффициент масштабирования для прокрутки
    trajectory_plot_side = 10
    robot_size = 0.5  # начальный размер робота
    counter = 0
    obstacle_width = 1
    obstacle_height = 1
    row_1 = 0.10
    row_2 = 0.06
    row_3 = 0.02
    init_column_1 = 0.02
    init_column_2 = 0.52
    
    obstacles_list = []
    obstacles_coords = []
    obstacles_distance = []
    obstacles_text = []
    robot_coords = []
    target_coords = []
    robot = False
    target = False
    robot_radius = 0.25
    robot_edge = 0.1
    target_radius = 0.1
    target_distance = False
    target_text = False
    
    buttons_1_list = ["obstacle", "robot", "target"]
    
    def __init__(self):
        
        self.trajectory = Trajectory(self)
        
        self.fig = plt.figure(figsize=(15, 10))
        self.gs = gridspec.GridSpec(4, 2, height_ratios=[1, 1, 1, 1])
        self.plt = plt
        self.linear_velocity_plot = self.plt.subplot(self.gs[0, 0])
        self.angle_velocity_plot = self.plt.subplot(self.gs[1, 0])
        self.left_velocity_plot = self.plt.subplot(self.gs[2, 0])
        self.right_velocity_plot = self.plt.subplot(self.gs[3, 0])
        self.trajectory_plot = self.plt.subplot(self.gs[:, 1])
        
        self.trajectory_plot.set_xlim([0, self.trajectory_plot_side])  # Длина шкалы по оси X
        self.trajectory_plot.set_ylim([0, self.trajectory_plot_side])  # Высота шкалы по оси Y
        self.trajectory_plot.set_aspect('equal') 
        
        self.linear_velocity_plot.set_title('Линейная скорость робота')
        self.angle_velocity_plot.set_title('Угловая скорость робота')
        self.left_velocity_plot.set_title('Угловая скорость левого колеса')
        self.right_velocity_plot.set_title('Угловая скорость правого колеса')
        self.trajectory_plot.set_title('Траектория')     

        self.plt.tight_layout()
        self.plt.subplots_adjust(bottom=0.16)
        
        #self.plt.get_current_fig_manager().window.showMaximized()
        self.set_buttons()
        

        self.fig.canvas.mpl_connect('button_press_event', self.drawing)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)  # Регистрация обработчика события прокрутки
        self.plt.show()
        
        
        
    def set_buttons(self):
        self.create_button_1("obstacle", 'Препятствие', self.init_column_2, self.row_1)
        self.create_button_2("delete",   'Удалить',     self.init_column_2, self.row_2)
        self.create_button_2("clear",    'Очистить',    self.init_column_2, self.row_3)
        self.create_entry("obstacle_width", 'Ширина препят.', self.obstacle_width, self.init_column_2 + self.button_width * 2, self.row_1)
        self.create_entry("obstacle_height", 'Высота препят.', self.obstacle_height, self.init_column_2 + self.button_width * 2, self.row_2)
        self.create_button_1("robot", 'Робот', self.init_column_2 + self.button_width * 2.5, self.row_1)
        self.create_button_1("target", 'Цель', self.init_column_2 + self.button_width * 2.5, self.row_2)
        self.create_button_2("step", 'Рассчитать шаг', self.init_column_2 + self.button_width * 3.8, self.row_1)
        self.create_button_2("trajectory", 'Рассчитать траек.', self.init_column_2 + self.button_width * 3.8, self.row_2)        
        
    def create_button_1(self, name, text, x, y):
        self.set_button(name, text, x, y)
        self.set_button_event_1(name)

    def create_button_2(self, name, text, x, y):
        self.set_button(name, text, x, y)
        self.set_button_event_2(name)

    def set_button(self, name, text, x, y):
        exec(f"self.{name}_button_position = self.plt.axes([{x}, {y}, {self.button_width}, {self.button_height}])")
        exec(f"self.{name}_button = Button(self.{name}_button_position, '{text}', color = self.color, hovercolor = self.hovercolor)")
        
    def set_button_event_1(self, name):
        self.temp_func = lambda event: self.button_event_1(name)
        exec(f"self.{name}_button.on_clicked(self.temp_func)")
        exec(f"self.is_{name}_button = False")

    def set_button_event_2(self, name):
        self.temp_func = lambda event: self.button_event_2(name)
        exec(f"self.{name}_button.on_clicked(self.temp_func)")


    def button_event_1(self, name):
        for item in self.buttons_1_list:
            if item == name:
                exec(f"self.is_{name}_button = not self.is_{name}_button")
                exec(f"self.is_temp_button = self.is_{name}_button")
                if self.is_temp_button:      
                    exec(f"self.{name}_button.color = self.hovercolor")
                    exec(f"self.{name}_button.hovercolor = self.color")
                else:                        
                    exec(f"self.{name}_button.color = self.color")
                    exec(f"self.{name}_button.hovercolor = self.hovercolor")
            else:
                exec(f"self.is_temp_button = self.is_{item}_button")
                if self.is_temp_button:      
                    exec(f"self.is_{item}_button = not self.is_{item}_button")
                    exec(f"self.{item}_button.color = self.color")
                    exec(f"self.{item}_button.hovercolor = self.hovercolor")
                    
                    
        self.fig.canvas.draw_idle()


    def button_event_2(self, name):

        if name == "delete":      
            if len(self.obstacles_list): 
                rect = self.obstacles_list[-1]
                self.obstacles_list.pop(-1)
                rect.remove()
        elif name == "clear":  
            for item in self.obstacles_list:
                item.remove()
            self.obstacles_list.clear()
            if self.robot:
                self.robot.remove()
                self.robot = False
            if self.target:
                self.target.remove()
                self.target = False
            self.remove_target_distance()
            self.remove_target_text()
            self.remove_obstacles_item()
                
        elif name == "step":
            if len(self.obstacles_coords) and self.robot_coords and self.target_coords:
                self.trajectory.calculate_step()
        
        elif name == "trajectory":
            pass

        
        self.fig.canvas.draw_idle()


    def drawing(self, event):
        if self.is_obstacle_button and event.inaxes == self.trajectory_plot:
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                self.obstacle_width = float(self.obstacle_width_entry.text)
                self.obstacle_height = float(self.obstacle_height_entry.text)
                rect = Rectangle((x - self.obstacle_width / 2, y - self.obstacle_height / 2), self.obstacle_width, self.obstacle_height, 
                                 fill=False, color='blue')
                
                x1 = x - self.obstacle_width / 2
                y1 = y - self.obstacle_height / 2
                x2 = x1 + self.obstacle_width
                y2 = y1
                x3 = x2
                y3 = y1 + self.obstacle_height
                x4 = x1
                y4 = y3
                
                self.obstacles_coords.append([x1, y1, x2, y2, x3, y3, x4, y4])
                
                self.obstacles_list.append(rect)
                self.trajectory_plot.add_patch(rect)
                self.fig.canvas.draw()
                
        elif self.is_robot_button and event.inaxes == self.trajectory_plot:
            self.remove_target_distance()
            self.remove_target_text()
            self.remove_obstacles_item()
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                #self.obstacle_width = float(self.obstacle_width_entry.text)
                #self.obstacle_height = float(self.obstacle_height_entry.text)
                if self.robot:
                    self.robot.remove()
                self.robot = Circle((x, y), self.robot_radius, color='red', fill=False)
                self.robot_coords.clear()
                self.robot_coords.append(x)
                self.robot_coords.append(y)
                self.trajectory_plot.add_patch(self.robot)
                self.fig.canvas.draw()
                
                
        elif self.is_target_button and event.inaxes == self.trajectory_plot:
            self.remove_target_distance()
            self.remove_target_text()
            self.remove_obstacles_item()
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                #self.obstacle_width = float(self.obstacle_width_entry.text)
                #self.obstacle_height = float(self.obstacle_height_entry.text)
                if self.target:
                    self.target.remove()
                self.target = Circle((x, y), self.target_radius, color='orange', fill=True)
                self.target_coords.clear()
                self.target_coords.append(x)
                self.target_coords.append(y)
                self.trajectory_plot.add_patch(self.target)
                self.fig.canvas.draw()
                
                
    def on_scroll(self, event):
        if event.inaxes == self.trajectory_plot:
            ax = self.trajectory_plot
            x_min, x_max = ax.get_xlim()
            y_min, y_max = ax.get_ylim()
            if event.button == 'up':
                # Увеличиваем масштаб
                ax.set_xlim([event.xdata - (event.xdata - x_min) / self.zoom_factor, event.xdata + (x_max - event.xdata) / self.zoom_factor])
                ax.set_ylim([event.ydata - (event.ydata - y_min) / self.zoom_factor, event.ydata + (y_max - event.ydata) / self.zoom_factor])
            elif event.button == 'down':
                # Уменьшаем масштаб
                ax.set_xlim([event.xdata - (event.xdata - x_min) * self.zoom_factor, event.xdata + (x_max - event.xdata) * self.zoom_factor])
                ax.set_ylim([event.ydata - (event.ydata - y_min) * self.zoom_factor, event.ydata + (y_max - event.ydata) * self.zoom_factor])
            self.fig.canvas.draw_idle()


    def create_entry(self, name, text, value, x, y):
        exec(f"self.{name}_entry_position = self.plt.axes([{x}, {y}, {self.entry_width}, {self.entry_height}])")
        exec(f"self.{name}_entry = TextBox(self.{name}_entry_position, '{text}', color = 'white', hovercolor ='grey')")
        exec(f"self.{name}_entry.set_val({value})")
        
    def create_distances_and_angles(self): #отрисовка расстояний от робота и углов до цели
        self.remove_target_distance()
        self.target_distance, = self.trajectory_plot.plot([self.robot_coords[0], self.target_coords[0]], [self.robot_coords[1], self.target_coords[1]],
                                                          linestyle='--', linewidth=1, color='orange')

        angle = round(math.degrees(self.trajectory.target_angle), 1)    
        #text = f'd {self.trajectory.target_distance}\na {angle}'
        #self.remove_target_text()
        #self.target_text = self.trajectory_plot.text(self.target_coords[0], self.target_coords[1], text, verticalalignment='bottom', horizontalalignment='right')
     
        self.remove_obstacles_item()
        for _distance, angle, coords in zip(self.trajectory.obstacles_distance, self.trajectory.obstacles_angle, self.trajectory.obstacles_coords):
            distance, = self.trajectory_plot.plot([self.robot_coords[0], coords[0]], [self.robot_coords[1], coords[1]],
                                                   linestyle='--', linewidth=1, color='blue')
            angle = round(math.degrees(angle), 1)
            _text = f'd {_distance}\na {angle}'
            text = self.trajectory_plot.text(coords[0], coords[1], _text, verticalalignment='bottom', horizontalalignment='right')
            self.obstacles_distance.append(distance)
            self.obstacles_text.append(text)
        
    def create_detour_point(self):  
        self.remove_target_distance()
        self.remove_target_text()
        self.remove_obstacles_item()

        self.target_distance, = self.trajectory_plot.plot([self.robot_coords[0], self.target_coords[0]], [self.robot_coords[1], self.target_coords[1]],
                                                          linestyle='--', linewidth=1, color='orange')

        angle = round(math.degrees(self.trajectory.target_angle), 1)    
        #text = f'd {self.trajectory.target_distance}\na {angle}'
        #self.remove_target_text()
        #self.target_text = self.trajectory_plot.text(self.target_coords[0], self.target_coords[1], text, verticalalignment='bottom', horizontalalignment='right')

#---------------------------------------------------------------------------------------
        for _kasat in (self.trajectory.point_track_list):
            kas, = self.trajectory_plot.plot([_kasat[0][0], _kasat[1][0]], [_kasat[0][1], _kasat[1][1]],
                                                   linewidth=1, color='red')
        
        for _ug in (self.trajectory.point_obstacle_list):
            ugol = Circle((_ug[0], _ug[1]), 0.3, color='blue',fill=False)
            self.trajectory_plot.add_patch(ugol)

#---------------------------------------------------------------------------------------
        
    def remove_target_distance(self):
        if self.target_distance:
            self.target_distance.remove()
            self.target_distance = False   

    def remove_target_text(self):
        if self.target_text:
            self.target_text.remove()
            self.target_text = False 

    def remove_obstacles_item(self):
        for distance, text in zip(self.obstacles_distance, self.obstacles_text):
            distance.remove()
            text.remove() 
        self.obstacles_distance.clear()
        self.obstacles_text.clear()

 


Plot()
