from tkinter import Tk, Canvas, Frame, Button, Scale
from math import pi, sqrt, cos, sin, acos
from time import time


WIDTH = 800
HEIGHT = 600


class Window:
    def __init__(self):
        self.__boids = []
        self.__is_run = 0
        self.__window = Tk()
        self.__window.resizable(width=False, height=False)
        self.__canvas = Canvas(
            self.__window,
            width=WIDTH,
            height=HEIGHT,
            bg="white"
        )
        self.__canvas.grid(row=0, column=0, sticky="nw")
        self.__canvas.bind("<Button-1>", self.__add_boids)
        self.__canvas.after(1, self.__run)
        frame = Frame(self.__window, borderwidth=0)
        frame.grid(row=0, column=1, sticky="nw")
        button = Button(
            frame,
            text="Remove all boids",
            command=self.__stop
        )
        button.grid(row=0, column=0, sticky="nw")
        self.__scale_add_number = Scale(
            frame,
            orient="horizontal",
            label="Add number :",
            length=200,
            resolution=1,
            from_=1,
            to=100
        )
        self.__scale_add_number.grid(row=1, column=0, sticky="nw")
        self.__scale_add_number.set(50)
        self.__scale_boids_fov = Scale(
            frame,
            orient="horizontal",
            label="Boids field of view :",
            length=200,
            resolution=1,
            from_=1,
            to=360
        )
        self.__scale_boids_fov.grid(row=3, column=0, sticky="nw")
        self.__scale_boids_fov.set(360)
        self.__scale_detection_radius = Scale(
            frame,
            orient="horizontal",
            label="Detection radius :",
            length=200,
            resolution=1,
            from_=1,
            to=100
        )
        self.__scale_detection_radius.grid(row=2, column=0, sticky="nw")
        self.__scale_detection_radius.set(40)
        self.__scale_flight_distance = Scale(
            frame,
            orient="horizontal",
            label="Flight distance :",
            length=200,
            resolution=1,
            from_=1,
            to=100
        )
        self.__scale_flight_distance.grid(row=4, column=0, sticky="nw")
        self.__scale_flight_distance.set(20)
        self.__scale_max_speed = Scale(
            frame,
            orient="horizontal",
            label="Max speed :",
            length=200,
            resolution=1,
            from_=1,
            to=10
        )
        self.__scale_max_speed.grid(row=5, column=0, sticky="nw")
        self.__scale_max_speed.set(5)
        self.__scale_separation_force = Scale(
            frame,
            orient="horizontal",
            label="Separation force :",
            length=200,
            resolution=0.01,
            from_=0.1,
            to=1
        )
        self.__scale_separation_force.grid(row=6, column=0, sticky="nw")
        self.__scale_separation_force.set(0.15)
        self.__scale_cohesion_force = Scale(
            frame,
            orient="horizontal",
            label="Cohesion force :",
            length=200,
            resolution=0.01,
            from_=0.1,
            to=1
        )
        self.__scale_cohesion_force.grid(row=7, column=0, sticky="nw")
        self.__scale_cohesion_force.set(0.1)
        self.__scale_alignment_force = Scale(
            frame,
            orient="horizontal",
            label="Alignment force :",
            length=200,
            resolution=0.01,
            from_=0.1,
            to=1
        )
        self.__scale_alignment_force.grid(row=8, column=0, sticky="nw")
        self.__scale_alignment_force.set(0.1)
        self.__window.mainloop()

    def __add_boids(self, event):
        for i in range(self.__scale_add_number.get()):
            self.__boids += [Boids(
                event.x,
                event.y+i,
                self.__scale_max_speed.get()
            )]
        if self.__is_run == 0:
            self.__is_run = 1
            self.__run()

    def __del_boids(self):
        del self.__boids[:]
        self.__canvas.delete("all")

    def __stop(self):
        self.__is_run = 0

    def __run(self):
        t = time()
        self.__canvas.delete("all")
        f_prop = 10
        s_prop = f_prop*1.12
        a = 153/180*pi
        u = Vector(1, 0)
        detection_radius = self.__scale_detection_radius.get()
        birds_fov = self.__scale_boids_fov.get()/360*pi
        flight_distance = self.__scale_flight_distance.get()
        max_speed = self.__scale_max_speed.get()
        separation_force = self.__scale_separation_force.get()
        cohesion_force = self.__scale_cohesion_force.get()
        alignment_force =self.__scale_alignment_force.get()
        for i in range(len(self.__boids)-1):
            self.__boids[i].reset()
            for j in range(i+1, len(self.__boids)):
                ij = self.__boids[j].location-self.__boids[i].location
                ji = ij*(-1)
                d = ij.norm()
                if d <= detection_radius:
                    if d == 0 or VectorsAngle(self.__boids[i].velocity, ij) <= birds_fov:
                        if d < flight_distance:
                            self.__boids[i].do_separation(ji)
                        else:
                            self.__boids[i].do_cohesion(self.__boids[j].location)
                        self.__boids[i].do_alignment(self.__boids[j].velocity)
                    if d == 0 or VectorsAngle(self.__boids[j].velocity, ji) <= birds_fov:
                        if d < flight_distance:
                            self.__boids[j].do_separation(ij)
                        else:
                            self.__boids[j].do_cohesion(self.__boids[i].location)
                        self.__boids[j].do_alignment(self.__boids[i].velocity)
            self.__boids[i].update(
                max_speed,
                separation_force,
                cohesion_force,
                alignment_force
            )
            if self.__boids[i].location.x < 0:
                self.__boids[i].location.x += WIDTH
            elif self.__boids[i].location.x >= WIDTH:
                self.__boids[i].location.x -= WIDTH
            if self.__boids[i].location.y < 0:
                self.__boids[i].location.y += HEIGHT
            elif self.__boids[i].location.y >= HEIGHT:
                self.__boids[i].location.y -= HEIGHT
            angle = VectorsAngle(u, self.__boids[i].velocity)
            if self.__boids[i].velocity.y < 0:
                angle *= -1
            xa = self.__boids[i].location.x+cos(angle)*f_prop
            ya = self.__boids[i].location.y+sin(angle)*f_prop
            xb = self.__boids[i].location.x+cos(angle+a)*s_prop
            yb = self.__boids[i].location.y+sin(angle+a)*s_prop
            xc = self.__boids[i].location.x+cos(angle-a)*s_prop
            yc = self.__boids[i].location.y+sin(angle-a)*s_prop
            self.__canvas.create_line(xa, ya, xb, yb)
            self.__canvas.create_line(xb, yb, xc, yc)
            self.__canvas.create_line(xc, yc, xa, ya)
        if self.__is_run:
            t = int((time()-t)*1000)
            if t < 16:
                self.__canvas.after(16-t, self.__run)
            else:
                self.__canvas.after(1, self.__run)
        else:
            self.__del_boids()


class Boids:
    def __init__(self, x, y, max_speed):
        self.location = Vector(x, y)
        self.velocity = Vector(max_speed, 0)
        self.__acceleration = Vector(0, 0)
        self.__n_separation = 0
        self.__separation = Vector(0, 0)
        self.__n_cohesion = 0
        self.__cohesion = Vector(0, 0)
        self.__n_alignment = 0
        self.__alignment = Vector(0, 0)

    def reset(self):
        self.__acceleration *= 0
        self.__n_separation = 0
        self.__separation *= 0
        self.__n_cohesion = 0
        self.__cohesion *= 0
        self.__n_alignment = 0
        self.__alignment *= 0

    def do_separation(self, u):
        norm = u.norm()
        if norm > 0:
            u.set_norm(1/norm)
            self.__n_separation += 1
            self.__separation += u

    def do_cohesion(self, u):
        self.__n_cohesion += 1
        self.__cohesion += u

    def do_alignment(self, u):
        self.__n_alignment += 1
        self.__alignment += u

    def update(self, max_speed, separation_force, cohesion_force, alignment_force):
        if self.__n_separation > 0:
            self.__separation /= self.__n_separation
            self.__separation.set_norm(max_speed)
            self.__separation.set_norm(separation_force)
            self.__acceleration += self.__separation
        if self.__n_cohesion > 0:
            self.__cohesion /= self.__n_cohesion
            self.__cohesion -= self.location
            self.__cohesion.set_norm(max_speed)
            self.__cohesion -= self.velocity
            self.__cohesion.set_norm(cohesion_force)
            self.__acceleration += self.__cohesion
        if self.__n_alignment > 0:
            self.__alignment /= self.__n_alignment
            self.__alignment.set_norm(max_speed)
            self.__alignment -= self.velocity
            self.__alignment.set_norm(alignment_force)
            self.__acceleration += self.__alignment
        else:
            self.velocity.set_norm(max_speed)
        self.velocity += self.__acceleration
        if self.velocity.norm() > max_speed:
            self.velocity.set_norm(max_speed)
        self.location += self.velocity


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def norm(self):
        return sqrt(self.x*self.x+self.y*self.y)

    def set_norm(self, n):
        if self.x == self.y == 0:
            self.x = n
            self.y = 0
        else:
            norm = self.norm()
            self.x *= n/norm
            self.y *= n/norm

    def __add__(u, v):
        return Vector(u.x+v.x, u.y+v.y)

    def __sub__(u, v):
        return Vector(u.x-v.x, u.y-v.y)

    def __mul__(self, n):
        return Vector(self.x*n, self.y*n)

    def __truediv__(self, n):
        return Vector(self.x/n, self.y/n)


def VectorsScalaire(u, v):
    return u.x*v.x+u.y*v.y


def VectorsAngle(u, v):
    a = VectorsScalaire(u, v)/(u.norm()*v.norm())
    if a > 1:
        a = 1
    elif a < -1:
        a = -1
    return acos(a)


window = Window()
