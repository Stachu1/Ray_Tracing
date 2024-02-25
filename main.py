import time, numpy as np
from PIL import Image
from colorama import Fore, init
init()



class Scene:
    def __init__(self, spheres, planes, light_sources):
        self.spheres = spheres
        self.planes = planes
        self.all_bodies = self.spheres + self.planes
        self.light_sources = light_sources        
      
        
#TODO: Rectangle intersection
# class Rectangle:
#     def __init__(self,):



class Texture:
    def __init__(self, filename, scale=1.0):
        self.img = Image.open(f"textures/{filename}").convert("RGB")
        self.width, self.height = self.img.size
        self.scale = scale



class Plane:
    def __init__(self, position, normal, color, reflectivity, texture=None):
        self.position = np.array(position)
        self.normal = self.normalize_vector(np.array(normal))
        self.color = np.array(color)
        self.reflectivity = reflectivity
        self.texture = texture
    
    def normalize_vector(self, vector):
     return vector/np.linalg.norm(vector)
    



class Light_source:
    def __init__(self, position, brightness, color, FOV, normal):
        self.position = np.array(position)
        self.brightness = brightness
        self.color = np.array(color)
        self.FOV = FOV
        self.normal = normal
    


class Sphere:
    def __init__(self, radius, position, color, reflectivity, texture=None):
        self.radius = radius
        self.position = np.array(position)
        self.color = np.array(color)
        self.reflectivity = reflectivity
        self.texture = texture



class Camera:
    def __init__(self, resolution, fov, position, rotation, gamma, max_reflections=1):
        self.resolution = np.array(resolution)
        self.fov = fov
        self.position = np.array(position)
        self.rotation = np.array(rotation)
        self.gamma = gamma
        self.max_reflections = max_reflections

        self.xMatrix = np.array(((1, 0, 0),
                            (0, np.cos(rotation[0]), -np.sin(rotation[0])),
                            (0, np.sin(rotation[0]), np.cos(rotation[0]))))

        self.yMatrix = np.array(((np.cos(rotation[1]), 0, np.sin(rotation[1])),
                            (0, 1, 0),
                            (-np.sin(rotation[1]), 0, np.cos(rotation[1]))))

        self.zMatrix = np.array(((np.cos(rotation[2]), -np.sin(rotation[2]), 0),
                            (np.sin(rotation[2]), np.cos(rotation[2]), 0),
                            (0, 0, 1)))


    def generate_ray(self, pixel, screen_distance):
        x = pixel[0] - self.resolution[0] / 2
        y = screen_distance
        z = -pixel[1] + self.resolution[1] / 2
        
        ray = np.array((x, y, z))

        ray = np.matmul(self.zMatrix, ray)      
        ray = np.matmul(self.yMatrix, ray)
        ray = np.matmul(self.xMatrix, ray)

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
            
            if np.arccos(np.dot(-light_ray, light_source.normal)) > light_source.FOV/2:
                continue
            
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
    
    
    def get_color_from_texture(self, body, intersection):
        if type(body) is Plane:
            plane = body
            if plane.normal[0] == 0 and plane.normal[2] == 0:
                x = intersection[0] - plane.position[0]
                y = intersection[2] - plane.position[2]
            elif plane.normal[1] == 0 and plane.normal[2] == 0:
                x = intersection[1] - plane.position[1]
                y = intersection[2] - plane.position[2]
            elif plane.normal[0] == 0 and plane.normal[1] == 0:
                x = intersection[0] - plane.position[0]
                y = intersection[1] - plane.position[1]
            else:
                return body.color
            
            x = x / plane.texture.scale
            y = y / plane.texture.scale
            
            if plane.normal[0] < 0:
                pixel_x = plane.texture.width - 1 - x % plane.texture.width
                pixel_y = plane.texture.height - 1 - y % plane.texture.height
                color = plane.texture.img.getpixel((int(pixel_x), int(pixel_y)))
                return np.array(color)
            if plane.normal[0] > 0:
                pixel_x = x % plane.texture.width
                pixel_y = plane.texture.height - 1 - y % plane.texture.height
                color = plane.texture.img.getpixel((int(pixel_x), int(pixel_y)))
                return np.array(color)
            
            if plane.normal[1] < 0:
                pixel_x = x % plane.texture.width
                pixel_y = plane.texture.height - 1 - y % plane.texture.height
                color = plane.texture.img.getpixel((int(pixel_x), int(pixel_y)))
                return np.array(color)
            if plane.normal[1] > 0:
                pixel_x = plane.texture.width - 1 - x % plane.texture.width
                pixel_y = plane.texture.height - 1 - y % plane.texture.height
                color = plane.texture.img.getpixel((int(pixel_x), int(pixel_y)))
                return np.array(color)
            
            if plane.normal[2] < 0:
                pixel_x = plane.texture.width - 1 - x % plane.texture.width
                pixel_y = plane.texture.height - 1 - y % plane.texture.height
                color = plane.texture.img.getpixel((int(pixel_x), int(pixel_y)))
                return np.array(color)
            if plane.normal[2] > 0:
                pixel_x = plane.texture.width - 1 - x % plane.texture.width
                pixel_y = y % plane.texture.height
                color = plane.texture.img.getpixel((int(pixel_x), int(pixel_y)))
                return np.array(color)
            
            
    def get_reflection_ray(self, ray, intersection, body):
        if type(body) is Sphere:
            normal_vector = self.normalize_vector(np.subtract(intersection, body.position))
        else:
            normal_vector = body.normal
        ray = self.normalize_vector(ray)
        reflection_ray = ray - 2 * np.dot(ray, normal_vector) * normal_vector
        return reflection_ray     
        
            
    def render_scene(self, scene):
        img = Image.new("RGB", tuple(self.resolution), (0,0,0))
        rays_length_array = np.zeros(self.resolution)
        print(f"{Fore.BLUE}Preparing rays:{Fore.RESET}")
        rays = self.generate_rays()
        
        time_start = time.time()
        print(f"{Fore.BLUE}\nRendering:{Fore.RESET}")
        for y, row in enumerate(rays):
            printProgressBar(y+1, self.resolution[1], time_start)
            for x, ray in enumerate(row):
                for body in scene.all_bodies:
                    intersection = self.check_for_ray_body_intersection(self.position, ray, body)
                    if intersection is not False:
                        ray_length = self.get_distance(self.position, intersection)
                        if rays_length_array[x][y] == 0 or rays_length_array[x][y] > ray_length:
                            rays_length_array[x][y] = ray_length
                            
                            if body.reflectivity is not False:
                                _ray = self.get_reflection_ray(ray, intersection, body)
                                for _body in scene.all_bodies:
                                    if _body == body:
                                        continue
                                    _intersection = self.check_for_ray_body_intersection(intersection, _ray, _body)
                                    if _intersection is not False:
                                        _ray_length = self.get_distance(_intersection, intersection)
                                        if rays_length_array[x][y] == 0 or rays_length_array[x][y] > _ray_length:
                                            rays_length_array[x][y] = _ray_length
                                            
                                            illumination = self.check_for_direct_illumination(_intersection, _body, scene)
                                            illumination = self.gamma_correction(illumination, self.gamma)
                                            if _body.texture is not None:
                                                color = self.get_color_from_texture(_body, _intersection) * illumination / 255
                                            else:
                                                color = _body.color * illumination / 255
                                            color = color * body.reflectivity + body.color * (1 - body.reflectivity)
                                                
                                            color = tuple(color.astype(int))
                                            img.putpixel((x, y), color)
                                
                            else:
                                illumination = self.check_for_direct_illumination(intersection, body, scene)
                                illumination = self.gamma_correction(illumination, self.gamma)
                                if body.texture is not None:
                                    color = self.get_color_from_texture(body, intersection) * illumination / 255
                                else:
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
        


#* (resolution, FOV, pos, rotation, gamma, max_reflections)
# camera = Camera((4096, 2160), np.pi/2, (0,0,30), (0, 0, 0), 2.4, 1)
camera = Camera((320, 180), np.pi/2, (0,0,30), (0, 0, 0), 2.4, 1)

#* (radius, pos, color, reflectivity)
sphere1 = Sphere(10, (-10,45,10), (0,0,255), 0.8)
sphere2 = Sphere(10, (20,35,10), (255,0,0), 0.8)

#* (pos, brightness, color, FOV, normal)
light_source1 = Light_source((0,30,50), 25, (255,255,255), np.pi*2, [0,0,-1])

#* (filename, scale)
texture_stone = Texture("stone.png", 0.5)
texture_diamond = Texture("diamond_ore.png", 0.5)
texture_redstone = Texture("redstone_ore.png", 0.5)
texture_lapis = Texture("lapis_ore.png", 0.5)
texture_checkerboard = Texture("checkerboard.png", 2)

#* (pos, norm, color, "texture")
plane1 = Plane((0,0,0), (0,0,1), (155,155,155), False, texture_checkerboard)
plane2 = Plane((-32,0,0), (1,0,0), (255,0,0), False, texture_redstone)
plane3 = Plane((0,64,0), (0,-1,0), (255,255,255), False, texture_diamond)
plane4 = Plane((32,0,0), (-1,0,0), (0,0,255), False, texture_lapis)
# plane5 = Plane((0,0,64), (0,0,-1), (255,255,255), False, texture_stone)
# plane6 = Plane((0,-32,0), (0,1,0), (255,255,255), False, texture_stone)

# plane1 = Plane((0,0,0), (0,0,1), (155,155,155), False)
# plane2 = Plane((-32,0,0), (1,0,0), (255,0,0), False)
# plane3 = Plane((0,64,0), (0,-1,0), (0,255,0), False)
# plane4 = Plane((32,0,0), (-1,0,0), (0,0,255), False)
plane5 = Plane((0,0,64), (0,0,-1), (255,255,255), False)
# plane6 = Plane((0,-0.1,0), (0,1,0), (255,255,255), False)

scene = Scene([sphere1, sphere2], [plane1, plane2, plane3, plane4, plane5], [light_source1])


img = camera.render_scene(scene)
img.save("render.png", format="png")
# img = img.resize((1200, 675))
img.show()