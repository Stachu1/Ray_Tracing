# Ray Tracing
A simple ray tracing engine made in Python.
This code was created to prove a point and so I could learn more about rendering, so it's not meant to be used for anything complex.
There is no support for STL or shapes other than spheres and planes.

## Download
```bash
git clone https://github.com/Stachu1/Ray_Tracing.git
```

## Configuration
```python
#* (resolution, FOV, pos, rotation, gamma, max_reflections)
camera = Camera((1280, 960), np.pi/2, (0,0,15), (-0.1, 0, 0), 2.4, 4)

#* (radius, pos, color, reflectivity)
sphere1 = Sphere(10, (-30,40,10), (0,0,255), False)
sphere2 = Sphere(10, (0,40,10), (0,255,0), False)
sphere3 = Sphere(10, (30,40,10), (255,0,0), False)

#* (pos, brightness, color, FOV, normal)
light_source1 = Light_source((0,0,20), 25, (255,255,255), np.pi*2, [0,0,-1])

#* (filename, scale)
texture_checkerboard = Texture("checkerboard.png", 2)

#* (pos, norm, color, "texture")
plane1 = Plane((0,0,0), (0,0,1), (155,155,155), False, texture_checkerboard)

scene = Scene([sphere1, sphere2, sphere3], [plane1], [light_source1])

img = camera.render_scene(scene)
img.save("render.png", format="png")
img.show()
```

# Output
1.![image](https://github.com/Stachu1/Ray_Tracing/assets/77758413/7f9b7894-1dbb-4172-90e6-171fc277db2c)
2.![image](https://github.com/Stachu1/Ray_Tracing/assets/77758413/c710fcd8-7657-4ac3-8386-965a9c3b4896)
3.![image](https://github.com/Stachu1/Ray_Tracing/assets/77758413/cbae484f-ef44-42d4-ba94-fb56b4f606b9)

