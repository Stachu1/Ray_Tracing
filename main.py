from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import time, os, numpy as np, random, sys, copy, pygame, pickle
from PIL import Image, ImageDraw
from struct import unpack
from colorama import Fore, init
init()




class Scene:
    def __init__(self, spheres):
        self.spheres = spheres
      
        

# class Rectangle:
#     def __init__(self,):
        


class Sphere:
    def __init__(self, radius, position, color, reflectivity):
        self.radius = radius
        self.position = position
        self.color = color
        self.reflectivity = reflectivity



class Camera:
    def __init__(self, resolution, fov, position, rotation):
        self.resolution = resolution
        self.fov = fov
        self.position = position
        self.rotation = rotation


    def generate_ray(self, pixel):        
        fov_y = self.fov * self.resolution[1] / self.resolution[0]
        
        angel_x = pixel[0] * self.fov / (self.resolution[0] - 1) - self.fov / 2
        angel_y = - pixel[1] * fov_y / (self.resolution[1] - 1) + fov_y / 2
        
        x = np.sin(angel_x)*np.cos(angel_y)
        y = np.cos(angel_x)*np.cos(angel_y)
        z = np.sin(angel_y)
        
        ray = np.array((x, y, z))
        return ray
    
    
    def generate_rays(self):
        rays = []
        for y_index in range(self.resolution[1]):
            row = []
            for x_index in range(self.resolution[0]):
                row.append(self.generate_ray((x_index, y_index)))
            rays.append(row)
        rays = np.array(rays)
        return rays
    
    
    def ray_sphere_intersection(self, ray, sphere):
        intersection_check = np.dot(ray, np.subtract(self.position, sphere.position))**2 - ((self.position[0] - sphere.position[0])**2 + (self.position[1] - sphere.position[1])**2 + (self.position[2] - sphere.position[2])**2) + sphere.radius**2
        if intersection_check < 0:
            return False
        if intersection_check == 0:
            d = -(np.dot(ray, np.subtract(self.position, sphere.position)))
            return np.add(self.position, ray * d)
        
        if intersection_check > 0:
            d1 = -(np.dot(ray, np.subtract(self.position, sphere.position))) - np.sqrt(intersection_check)
            d2 = -(np.dot(ray, np.subtract(self.position, sphere.position))) + np.sqrt(intersection_check)
            if d1 <= d2:
                d = d1
            else:
                d = d2
            return np.add(self.position, ray * d)
    
    
    def render_scene(self, scene):
        img = Image.new("RGB", tuple(self.resolution), (0,0,0))
        draw = ImageDraw.Draw(img)
        
        for sphere in scene.spheres:
            rays = self.generate_rays()
            for y, row in enumerate(rays):
                for x, ray in enumerate(row):
                    if self.ray_sphere_intersection(ray, sphere) is not False:
                        img.putpixel((x, y), sphere.color)
        return img
        
        
        





camera = Camera(np.array((160, 90)), np.pi/2, np.array((0,0,0)), np.array((0,0,0)))

sphere1 = Sphere(5, np.array((-10,10,0)), (255,0,0), 0)
sphere2 = Sphere(5, np.array((-5,15,0)), (0,0,255), 0)
sphere3 = Sphere(5, np.array((5,20,0)), (0,255,0), 0)
sphere4 = Sphere(5, np.array((0,20,10)), (255,0,255), 0)

scene = Scene([sphere1, sphere2, sphere3, sphere4])



img = camera.render_scene(scene)
img = img.resize((1200, 675))
img.show()



