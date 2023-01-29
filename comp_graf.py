import time
import numpy as np
import matplotlib.pyplot as plt


def normalize(vector):
    return vector / np.linalg.norm(vector)


def inverseVector(vector, axis):
    return vector - 2 * np.dot(vector, axis) * axis


def intersection_sphere(center, radius, ray_origin, ray_direction):
    b = 2 * np.dot(ray_direction, ray_origin - center)
    c = np.linalg.norm(ray_origin - center) ** 2 - radius ** 2

    delta = b ** 2 - 4 * c

    if delta > 0:
        t1 = (-b + np.sqrt(delta)) / 2
        t2 = (-b - np.sqrt(delta)) / 2
        if t1 > 0 and t2 > 0:
            return min(t1, t2)

    return None

def cylinder_intersection(cylinder_center, cylinder_axis, cylinder_radius, plane_normal, plane_point):
    # Vector from cylinder center to plane point
    d = plane_point - cylinder_center
    # Project d onto cylinder axis
    projection = np.dot(d, cylinder_axis) * cylinder_axis
    # Vector from projection to plane point
    e = d - projection
    # Distance from cylinder center to plane
    distance = np.linalg.norm(e)
    if distance > cylinder_radius:
        # No intersection
        return None
    else:
        # Distance from projection to intersection point
        h = np.sqrt(cylinder_radius**2 - distance**2)
        # Intersection point on cylinder axis
        P = cylinder_center + projection - h * cylinder_axis
        # Intersection points on cylinder
        Q1 = P + e
        Q2 = P - e
        return Q1, Q2  


def cube_intersect(center, size, ray_origin, ray_direction):
    # calculate the vectors for each side of the cube
    tmin = (center - size/2 - ray_origin) / ray_direction
    tmax = (center + size/2 - ray_origin) / ray_direction
    tmin, tmax = np.minimum(tmin, tmax), np.maximum(tmin, tmax)
    # find the maximum of the minimum intersection distances
    t0 = np.maximum(tmin[0], np.maximum(tmin[1], tmin[2]))
    # find the minimum of the maximum intersection distances
    t1 = np.minimum(tmax[0], np.minimum(tmax[1], tmax[2]))
    # if t0 is greater than t1, no intersection
    if t0 > t1:
        return None
    return t0


def intersection_plane(normal, point, ray_origin, ray_direction, size):
    denom = np.dot(normal, ray_direction)
    if abs(denom) > 1e-6:
        t = np.dot((point - ray_origin), normal) / denom
        if t >= 0:
            intersection = ray_origin + t * ray_direction
# check if intersection point is within the size of the plane
            if abs(intersection[0] - point[0]) < size/2 and abs(intersection[1] - point[1]) < size/2:
                return t
    return None


def min_distance_object(objects, ray_origin, ray_direction):
    distances = []

    for geometry in objects:
        if geometry['geometry'] == 'sphere':
            distances.append(intersection_sphere(geometry['center'], geometry['radius'], ray_origin, ray_direction))
        if geometry['geometry'] == 'plane':
            distances.append(intersection_plane(geometry['normal'], geometry['point'], ray_origin, ray_direction, geometry['size']))
        if geometry['geometry'] == 'cube':
            distances.append(cube_intersect(geometry['center'], geometry['size'], ray_origin, ray_direction))
        if geometry['geometry'] == 'cylinder':
            distances.append(cylinder_intersection(geometry['center'], geometry['axis'], geometry['radius'], geometry['normal'], geometry['point']))
        

    min_object = None
    min_distance = np.inf
    for i, d in enumerate(distances):
        if d and d < min_distance:
            min_distance = d
            min_object = objects[i]
    return min_object, min_distance


def render(viewpoint):
    start = time.time()
    width = width_screen
    height = height_screen

    max_depth = 3

    camera = np.array([viewpoint['x'], viewpoint['y'], viewpoint['z']])
    ratio = float(width) / height
    screen = (-1, 1 / ratio, 1, -1 / ratio)

    light = {
        'position': np.array([4, 2, 5]),
        'ambient': np.array([1, 1, 1]),
        'diffuse': np.array([1, 1, 1]), 
        'specular': np.array([1, 1, 1]),
    }

    image = np.zeros((height, width, 3))
    for i, y in enumerate(np.linspace(screen[1], screen[3], height)):
        for j, x in enumerate(np.linspace(screen[0], screen[2], width)):

            pixel = np.array([x, y, 0])
            origin = camera
            direction = normalize(pixel - origin)

            color = np.zeros((3))
            reflection = 1

            for k in range(max_depth):
                # check for intersections
                min_object, min_distance = min_distance_object(geometries, origin, direction)
                if min_object is None:
                    break

                intersection = origin + min_distance * direction
                
                if min_object['geometry'] == 'sphere':
                    normal_to_surface = normalize(intersection - min_object['center'])
                elif min_object['geometry'] == 'plane':
                    normal_to_surface = min_object['normal']
                elif min_object['geometry'] == 'cylinder':
                    normal_to_surface = normalize(intersection - min_object['center'])
                    Q1, Q2 = cylinder_intersection(min_object['center'], min_object['axis'], min_object['radius'], normal_to_surface, direction, intersection)
                    if np.linalg.norm(intersection - Q1) < np.linalg.norm(intersection - Q2):
                        normal_to_surface = normalize(intersection - Q1)
                    else:
                        normal_to_surface = normalize(intersection - Q2)

                shifted_point = intersection + 1e-5 * normal_to_surface
                intersection_to_light = normalize(light['position'] - shifted_point)

                _, min_distance = min_distance_object(geometries, shifted_point, intersection_to_light)
                intersection_to_light_distance = np.linalg.norm(light['position'] - intersection)
                is_shadowed = min_distance < intersection_to_light_distance

                if is_shadowed and min_object['geometry'] != 'cube':
                    break


                illumination = np.zeros((3))

                # ambiente
                illumination += min_object['ambient'] * light['ambient']

                # difusão
                illumination += min_object['diffuse'] * light['diffuse'] * np.dot(
                    intersection_to_light, normal_to_surface)

                # especular
                intersection_to_camera = normalize( camera - intersection)
                H = normalize(
                    intersection_to_light + intersection_to_camera)
                illumination += min_object['specular'] * light['specular'] * np.dot(
                    normal_to_surface, H) ** (min_object['shininess'] / 4)

                # reflexo
                color += reflection * illumination
                reflection *= min_object['reflection']

                origin = shifted_point
                direction = inverseVector(direction, normal_to_surface)

            image[i, j] = np.clip(color, 0, 1)

        print("%d/%d" % (i + 1, height))
    end = time.time()
    print(f'tempo corrido do script: {round(end - start)} segundos')
    plt.imsave('image.png', image)


width_screen = int(input('digite a largura da imagem '))
height_screen = int(input('digite a altura da imagem '))
pointview = list(map(float, input('digite o ponto de visao da camera(ex: 0.5, 0, 1.2): ').split(' ')))

# posição(x, y, z) | raio do circulo | luz ambiente | luz difusa | luz especular | brilho | reflexão
geometries = [
    # { 'geometry': 'cube', 'center': np.array([0.4, -0.50, -.5]), 'radius': 0.15, 'ambient': np.array([255, 0, 0]), 'diffuse': np.array([0, 0.6, 0]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5 },
    {'geometry': 'sphere', 'center': np.array([0.3, .2, .8]), 'radius': 0.15, 'ambient': np.array(
        [0, 0, 255]), 'diffuse': np.array([0, 0, 255]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0.5},
    {'geometry': 'plane','size': 1.2,  'normal': np.array([0.1, 0.1, 1]), 'point': np.array([0, 0, 0]), 'ambient': np.array(
        [0.1, 0.1, 255]), 'diffuse': np.array([0.40, 0.22, 0.11]), 'specular': np.array([1, 1, 1]), 'shininess': 100, 'reflection': 0},
    { 
	    'geometry': 'cube', 
		'center': np.array([0.3, .6, .8]),
		'size': 0.20, 
		'ambient': np.array([255, 0, 0]), 
		'diffuse': np.array([0, 0.6, 0]), 
		'specular': np.array([1, 1, 1]), 
		'shininess': 100,
        'reflection' : 0.5,
	},                
]

while True:
    op = int(input('1 - adicionar forma ao plano | 2 - gerar imagem '))
    if op == 1:
        position = list(
            map(float, input('digite a posicao(x y z): ').split(' ')))
        radius = float(input('digite o tamanho da forma(ex: 0.15): '))
        ambient = list(
            map(float, input('digite a luz ambiente(x y z): ').split(' ')))
        diffuse = list(
            map(float, input('digite a luz difusa(x y z): ').split(' ')))
        specular = list(
            map(int, input('digite a luz especular(x y z): ').split(' ')))
        shininess = float(input('digite o brilho da forma(ex: 100): '))
        reflection = float(input('digite o brilho da forma(ex: 0.5): '))

        geometries.append(
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
