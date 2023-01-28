import numpy as np
import matplotlib.pyplot as plt

def normalize(vector):
    return vector / np.linalg.norm(vector)

def reflected(vector, axis):
    return vector - 2 * np.dot(vector, axis) * axis

def sphere_intersect(center, radius, ray_origin, ray_direction):
    b = 2 * np.dot(ray_direction, ray_origin - center)
    c = np.linalg.norm(ray_origin - center) ** 2 - radius ** 2
    delta = b ** 2 - 4 * c
    if delta > 0:
        t1 = (-b + np.sqrt(delta)) / 2
        t2 = (-b - np.sqrt(delta)) / 2
        if t1 > 0 and t2 > 0:
            return min(t1, t2)
    return None

def nearest_intersected_object(objects, ray_origin, ray_direction):
    distances = [sphere_intersect(obj['center'], obj['radius'], ray_origin, ray_direction) for obj in objects]
    nearest_object = None
    min_distance = np.inf
    for index, distance in enumerate(distances):
        if distance and distance < min_distance:
            min_distance = distance
            nearest_object = objects[index]
    return nearest_object, min_distance


def render(viewpoint):
    width = width_screen
    height = height_screen

    max_depth = 2

    camera = np.array([viewpoint['x'], viewpoint['y'], viewpoint['z']])
    ratio = float(width) / height
    screen = (-1, 1 / ratio, 1, -1 / ratio)

    light = { 'position': np.array([5, 5, 5]), 'ambient': np.array([1, 1, 1]), 'diffuse': np.array([1, 1, 1]), 'specular': np.array([1, 1, 1]) }


    image = np.zeros((height, width, 3))
    for i, y in enumerate(np.linspace(screen[1], screen[3], height)):
        for j, x in enumerate(np.linspace(screen[0], screen[2], width)):
            # screen is on origin
            pixel = np.array([x, y, 0])
            origin = camera
            direction = normalize(pixel - origin)

            color = np.zeros((3))
            reflection = 1

            for k in range(max_depth):
                # check for intersections
                nearest_object, min_distance = nearest_intersected_object(objects, origin, direction)
                if nearest_object is None:
                    break

                intersection = origin + min_distance * direction
                normal_to_surface = normalize(intersection - nearest_object['center'])
                shifted_point = intersection + 1e-5 * normal_to_surface
                intersection_to_light = normalize(light['position'] - shifted_point)

                _, min_distance = nearest_intersected_object(objects, shifted_point, intersection_to_light)
                intersection_to_light_distance = np.linalg.norm(light['position'] - intersection)
                is_shadowed = min_distance < intersection_to_light_distance

                if is_shadowed:
                    break

                illumination = np.zeros((3))

                # ambiente
                illumination += nearest_object['ambient'] * light['ambient']

                # difusão
                illumination += nearest_object['diffuse'] * light['diffuse'] * np.dot(intersection_to_light, normal_to_surface)

                # especular
                intersection_to_camera = normalize(camera - intersection)
                H = normalize(intersection_to_light + intersection_to_camera)
                illumination += nearest_object['specular'] * light['specular'] * np.dot(normal_to_surface, H) ** (nearest_object['shininess'] / 4)

                # reflexo
                color += reflection * illumination
                reflection *= nearest_object['reflection']

                origin = shifted_point
                direction = reflected(direction, normal_to_surface)

            image[i, j] = np.clip(color, 0, 1)
        print("%d/%d" % (i + 1, height))

    plt.imsave('image.png', image)

width_screen = int(input('digite a largura da imagem '))
height_screen = int(input('digite a altura da imagem '))
pointview = list(map(float, input('digite o ponto de visao da camera(ex: 0.5, 0, 1.2): ').split(' ')))

# posição(x, y, z) | raio do circulo | luz ambiente | luz difusa | luz especular | brilho | reflexão
objects = [	
 	#{ 'geometry': 'cube', 'center': np.array([0.4, -0.50, -.5]), 'radius': 0.15, 'ambient': np.array([255, 0, 0]), 'diffuse': np.array([0, 0.6, 0]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 },
	{ 'geometry': 'sphere', 'center': np.array([-0.3, 0, 0]), 'radius': 0.15, 'ambient': np.array([0, 0, 255]), 'diffuse': np.array([0, 0, 255]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 },	
	{ 'geometry': 'plane', 'center': np.array([0, -9000, 0]), 'radius': 9000 - 0.7, 'ambient': np.array([0.1, 0.1, 0]), 'diffuse': np.array([0.6, 0.6, 0.6]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 },
]

while True:
    op = int(input('1 - adicionar forma ao plano | 2 - gerar imagem '))  
    if op == 1:
        position = list(map(float, input('digite a posicao(x y z): ').split(' ')))
        radius = float(input('digite o tamanho da forma(ex: 0.15): '))
        ambient = list(map(float, input('digite a luz ambiente(x y z): ').split(' ')))
        diffuse = list(map(float, input('digite a luz difusa(x y z): ').split(' ')))
        specular = list(map(int, input('digite a luz especular(x y z): ').split(' ')))
        shininess = float(input('digite o brilho da forma(ex: 100): '))
        reflection = float(input('digite o brilho da forma(ex: 0.5): '))

        objects.append(
            {
                'geometry': 'sphere', 
                'center': np.array(position), 
                'radius': radius, 
                'ambient': np.array(ambient), 
                'diffuse': np.array(diffuse), 
                'specular': np.array(specular), 
                'shininess': shininess, 
                'reflection': reflection
            },	
        )

    elif op == 2:
        viewpoint = {
            'x': pointview[0],
            'y': pointview[1],
            'z': pointview[2],
        }
        render(viewpoint)
        break
    else:
        break

