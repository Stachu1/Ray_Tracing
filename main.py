from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import time, os, numpy as np, random, sys, copy, pygame, pickle
from PIL import Image, ImageDraw
from struct import unpack
from colorama import Fore, init
init()




class Scene:
    def __init__(self, spheres, planes, light_sources):
        self.spheres = spheres
        self.planes = planes
        self.all_bodies = self.spheres + planes
        self.light_sources = light_sources
        
      
        
#TODO: Rectangle intersection
# class Rectangle:
#     def __init__(self,):



class Plane:
    def __init__(self, position, normal, color):
        self.position = np.array(position)
        self.normal = self.normalize_vector(np.array(normal))
        self.color = np.array(color)
    
    def normalize_vector(self, vector):
     return vector/np.linalg.norm(vector)
    



class Light_source:
    def __init__(self, position, brightness, color):
        self.position = np.array(position)
        self.brightness = brightness
        self.color = np.array(color)
    


class Sphere:
    def __init__(self, radius, position, color, reflectivity):
        self.radius = radius
        self.position = np.array(position)
        self.color = np.array(color)
        self.reflectivity = reflectivity



class Camera:
    def __init__(self, resolution, fov, position, rotation, gamma):
        self.resolution = np.array(resolution)
        self.fov = fov
        self.position = np.array(position)
        self.rotation = np.array(rotation)
        self.gamma = gamma


    def generate_ray(self, pixel, screen_distance):
        x = pixel[0] - self.resolution[0] / 2
        y = screen_distance
        z = -pixel[1] + self.resolution[1] / 2
        
        # ray = np.array((x, y, z))
        # ray = self.normalize_vector(ray)
        
        # x = ray[0]
        # y = ray[1]
        # z = ray[2]
        
        # angle_x = np.arccos(z)
        # angle_z = np.arctan2(y, x)
        
        # x = np.sin(angle_x)*np.cos(angle_z)
        # y = np.cos(angle_x)*np.cos(angle_z)
        # z = np.sin(angle_z)
        
        ray = np.array((x, y, z))
        ray = self.normalize_vector(ray)
        return ray
    
    
    def generate_rays(self):
        time_start = time.time()
        screen_distance = np.cos(self.fov/2) * self.resolution[0] / 2
        rays = []
        for y_index in range(self.resolution[1]):
            printProgressBar(y_index+1, self.resolution[1], time_start)
            row = []
            for x_index in range(self.resolution[0]):
                row.append(self.generate_ray((x_index, y_index), screen_distance))
            rays.append(row)
        rays = np.array(rays)
        
        return rays
    
    
    def check_for_ray_sphere_intersection(self, position, ray, sphere):
        intersection_check = np.power(np.dot(ray, np.subtract(position, sphere.position)), 2) - np.power(self.get_distance(position, sphere.position), 2) + sphere.radius**2
        
        if intersection_check < 0:
            return False
        if intersection_check == 0:
            d = -(np.dot(ray, np.subtract(position, sphere.position)))
            return np.add(position, ray * d)
        
        if intersection_check > 0:
            d1 = -(np.dot(ray, np.subtract(position, sphere.position))) - np.sqrt(intersection_check)
            d2 = -(np.dot(ray, np.subtract(position, sphere.position))) + np.sqrt(intersection_check)
            if np.abs(d1) <= np.abs(d2):
                d = d1
            else:
                d = d2
            if d > 0:
                return np.add(position, ray * d)
            else:
                return False
    
    
    def check_for_ray_plane_intersection(self, position, ray, plane):
        intersection_check = - np.dot(ray, plane.normal)
        
        if intersection_check != 0:
            n = np.subtract(position[0], plane.position[0]) * plane.normal[0] + np.subtract(position[1], plane.position[1]) * plane.normal[1] + np.subtract(position[2], plane.position[2]) * plane.normal[2]
            d = n / intersection_check
            if d >= 0:
                return np.add(position, ray * d)
        return False
    
    
    def check_for_ray_body_intersection(self, position, ray, body):
        body_type = type(body)
        if body_type is Sphere:
            return self.check_for_ray_sphere_intersection(position, ray, body)
        if body_type is Plane:
            return self.check_for_ray_plane_intersection(position, ray, body)
    
    
    def get_distance(self, p1, p2, sqrt=True):
        d = np.power(np.subtract(p1[0], p2[0]), 2) + np.power(np.subtract(p1[1], p2[1]), 2) + np.power(np.subtract(p1[2], p2[2]), 2)
        if sqrt:
            d = np.sqrt(d)
        return d
    
    
    def get_vector_length(self, vector):
        return np.sqrt(np.dot(vector, vector))
    
    
    def normalize_vector(self, vector):
     return vector/np.linalg.norm(vector)
    
    
    def check_for_direct_illumination(self, position, body, scene):
        illumination = np.array((0,0,0))
        for light_source in scene.light_sources:
            light_ray = np.subtract(light_source.position, position)
            light_ray = self.normalize_vector(light_ray)
            
            ray_blocked = False
            for sphere in scene.spheres:
                intersection = self.check_for_ray_body_intersection(position, light_ray, sphere)
                if intersection is not False and self.get_distance(sphere.position, light_source.position) < self.get_distance(position, light_source.position):
                    ray_blocked = True
                    break
                
            if not ray_blocked:
                if type(body) is Sphere:
                    normal = np.subtract(position, body.position)
                else:
                    normal = body.normal
                normal = self.normalize_vector(normal)
                
                brightness_multiplier = np.dot(light_ray, normal) / (self.get_vector_length(light_ray) * self.get_vector_length(normal))
                if brightness_multiplier > 0:
                    brightness_multiplier = brightness_multiplier * (np.power(light_source.brightness / self.get_distance(position, light_source.position), 2))
                    illumination = illumination + light_source.color * brightness_multiplier
        if illumination.max() > 255:
            illumination = illumination / illumination.max() * 255
        return illumination

    
    def gamma_correction(self, color, gamma):
        if type(color) == np.ndarray:
            color = np.power(color/255, 1/gamma) * 255
        else:
            color = np.power(color, 1/gamma)
        return color
        
            
    def render_scene(self, scene):
        img = Image.new("RGB", tuple(self.resolution), (0,0,0))
        rays_length_array = np.zeros(self.resolution)
        print(f"{Fore.CYAN}Preparing rays:{Fore.RESET}")
        rays = self.generate_rays()
        
        time_start = time.time()
        print(f"{Fore.CYAN}\nRendering:{Fore.RESET}")
        for y, row in enumerate(rays):
            printProgressBar(y+1, self.resolution[1], time_start)
            for x, ray in enumerate(row):
                for body in scene.all_bodies:
                    intersection = self.check_for_ray_body_intersection(self.position, ray, body)
                    if intersection is not False:
                        ray_length = self.get_distance(self.position, intersection)
                        if rays_length_array[x][y] == 0 or rays_length_array[x][y] > ray_length:
                            rays_length_array[x][y] = ray_length
                            
                            illumination = self.check_for_direct_illumination(intersection, body, scene)
                            illumination = self.gamma_correction(illumination, self.gamma)
                            color = body.color * illumination / 255
                            color = tuple(color.astype(int))
                            img.putpixel((x, y), color)
        return img
        


def printProgressBar (progress, total, time_start):
    percent = 100 * (progress / total)
    bar = "█" * int(percent) + "-" * (100 - int(percent))
    time_total = time.time() - time_start
    if time_total >= 60:
        time_total = f"{int(time_total // 60)}m {(time_total % 60):.2f}s"
    else:
        time_total = f"{time_total:.2f}s"
        
    time_left = (time.time() - time_start) / (percent / 100) - (time.time() - time_start)
    if time_left >= 60:
        time_left = f"{int(time_left // 60)}m {(time_left % 60):.2f}s"
    else:
        time_left = f"{time_left:.2f}s"
    print(f"{bar} {percent:.2f}%   t: {time_total}   eta: " + time_left, end="     \r")
    if progress == total: 
        print(f"{Fore.GREEN}{bar} {percent:.2f}%   t: {time_total}   eta: --      {Fore.RESET}")    
        



camera = Camera((1600, 900), np.pi/2, (0,0,5), 0, 2.4)
# camera = Camera((320, 180), np.pi/2, (0,0,30), 0, 2.4)

sphere1 = Sphere(15, (-15,45,15), (255,255,0), 0)
sphere2 = Sphere(15, (15,35,15), (0,255,255), 0)
sphere3 = Sphere(5, (10,50,5), (0,0,255), 0)

light_source1 = Light_source((0,30,50), 25, (255,255,255))
# light_source2 = Light_source((0,0,100), 80, (255,255,255))
# light_source3 = Light_source((-30,0,100), 80, (255,255,255))

plane1 = Plane((0,0,0), (0,0,1), (155,155,155))
plane2 = Plane((-30,0,0), (1,0,0), (255,0,0))
plane3 = Plane((0,60,0), (0,-1,0), (255,255,255))
plane4 = Plane((30,0,0), (-1,0,0), (0,0,255))
plane5 = Plane((0,0,60), (0,0,-1), (255,255,255))

scene = Scene([sphere1, sphere2], [plane1, plane2, plane3, plane4, plane5], [light_source1])



img = camera.render_scene(scene)
img.save("render.png", format="png")
img = img.resize((1200, 675))
img.show()