import cv2
import numpy as np

# Diccionario ArUco 7x7 con 1000 marcadores
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)

# Tamaño del marcador (con bordes)
marker_size_px = 140  # tamaño en píxeles de la imagen generada
marker_cells = 7      # número de celdas internas (sin contar bordes)
border_cells = 1      # borde ArUco siempre es de 1 celda
total_cells = marker_cells + 2 * border_cells
cell_size = marker_size_px // total_cells  # tamaño en píxeles de cada celda

with open("aruco7x7.dict", "w") as f:
    f.write("name ARUCO7X7\n")
    f.write("nbits 49\n")

    for marker_id in range(1000):
        # Generar la imagen del marcador
        marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)

        bits = ""

        # Iterar por las 7x7 celdas internas (ignorar los bordes)
        for y in range(border_cells, border_cells + marker_cells):
            for x in range(border_cells, border_cells + marker_cells):
                # Obtener región de la celda
                y_start = y * cell_size
                y_end = (y + 1) * cell_size
                x_start = x * cell_size
                x_end = (x + 1) * cell_size
                cell = marker_img[y_start:y_end, x_start:x_end]

                # Promediar el color (0=negro, 255=blanco)
                mean_val = np.mean(cell)
                bit = '0' if mean_val < 128 else '1'  # <- Invertido aquí
                bits += bit

        # Validar que se hayan generado 49 bits exactos
        assert len(bits) == 49, f"Error en ID {marker_id}: {len(bits)} bits"
        f.write(bits + "\n")
