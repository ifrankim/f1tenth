import cv2

      
def get_color_at_position(image, position):
    x, y = position
    color = image[y, x]  # OpenCV usa o formato BGR, então a ordem dos canais é invertida
    return color

map_image = cv2.imread('/home/rilbasestation02/sim_ws/src/f1tenth_gym_ros/maps/icra_2023_mod_color.jpeg')
car_position = (200, 250)  # Suponha que esta é a posição do carro mapeada para a imagem
car_color = get_color_at_position(map_image, car_position)
print(len(map_image))
print("Cor do carro:", car_color)


def convert_position_to_pixel(image, car_position):
    pixel_scale = 0.05  # 1 px : 0.05 m
    car_origin = (10.049340820312501, 4.060702514648438) # canto inferior esquerdo
    car_origin = (car_origin[0], len(image)*pixel_scale - car_origin[1]) # canto inferior direito (mesmo da imagem)
    
    # Converter coordenadas do carro para coordenadas do mapa
    map_position = (car_origin[0] + car_position[0], car_origin[1] - car_position[1])
    
    # Converter coordenadas do mapa para pixels na imagem
    pixel_position = (int(map_position[0] / pixel_scale), int(map_position[1] / pixel_scale))
    return pixel_position

def get_color_at_position(position):
    map_image = cv2.imread('/home/rilbasestation02/sim_ws/src/f1tenth_gym_ros/maps/icra_2023_mod_color.jpeg')
    x, y = convert_position_to_pixel(map_image, position)
    color = map_image[y, x]  # OpenCV usa o formato BGR, então a ordem dos canais é invertida
    return color
# Exemplo de uso
car_position_meters = (2.5, 3.0)  # Suponha que a posição do carro seja (2.5 m, 3.0 m)
car_position_pixel = get_color_at_position(car_position_meters)
print("Posição do carro em pixels:", car_position_pixel)

