def mostrar_camara_realsense(model):
    # Configurar el pipeline y el stream de la cámara RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            # Esperar a que haya frames disponibles
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            height, width, _ = color_image.shape
            center_x, center_y = int(width / 2), int((height / 2))
            cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

            distance = depth_frame.get_distance(center_x, center_y)

            H = distance*np.tan(23) #23
            V = distance*np.tan(83.96) 
            V = np.abs(V)
            # Mostrar la imagen con el punto dibujado y la distancia
            cv2.putText(color_image, f"Distance: {distance:.2f} m, H: {H:.2f} m, V: {V:.2f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('RealSense Camera', color_image)

            resized_frame = cv2.resize(color_image, (1020, 720))
            
            # Aplicar el modelo de detección
            results = model(resized_frame, conf=0.7, classes=[0, 2, 3, 4])
            annotated_frame = results[0].plot()

            # Procesar los resultados del modelo como en tu función original
            boxes = results[0].boxes
            for box in boxes:
                if 0 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    center_x = int((coordinates[0] + coordinates[2]) / 2)
                    center_y = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Bisturí x pixels: {center_x}")
                    print(f"Bisturí y pixels: {center_y}")
                    print(f"Bisturí x cm: {mapValue(center_x, 0, 1020, 0, H)}")
                    print(f"Bisturí y cm: {mapValue(center_y, 0, 720, 0, V)}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112 ,255)
                elif 2 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    center_x = int((coordinates[0] + coordinates[2]) / 2)
                    center_y = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Pinzas x: {center_x}")
                    print(f"Pinzas y: {center_y}")
                    print(f"Pqinzas x cm: {mapValue(center_x, 0, 1020, 0, H)}")
                    print(f"Pinzas y cm: {mapValue(center_y, 0, 720, 0, V)}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112 ,255)
                elif 3 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    center_x = int((coordinates[0] + coordinates[2]) / 2)
                    center_y = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Tijeras curvas x: {center_x}")
                    print(f"Tijeras curvas y: {center_y}")
                    print(f"Tijeras curvas x cm: {mapValue(center_x, 0, 1020, 0, H)}")
                    print(f"Tijeras curvas y cm: {mapValue(center_y, 0, 720, 0, V)}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112, 255)
                elif 4 in box.cls:
                    coordinates = box.xyxy.squeeze().tolist()
                    center_x = int((coordinates[0] + coordinates[2]) / 2)
                    center_y = int((coordinates[1] + coordinates[3]) / 2)
                    print(f"Tijeras rectas x cm: {mapValue(center_x, 0, 1020, 0, H)}")
                    print(f"Tijeras rectas y cm: {mapValue(center_y, 0, 720, 0, V)}")
                    calculateCenter(annotated_frame,center_x, center_y, 31, 112, 255)

            # Mostrar la imagen anotada
            cv2.imshow('Resultado', annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Presiona 'q' para salir
                break
    finally:
        # Detener la captura y cerrar todas las ventanas
        pipeline.stop()
        cv2.destroyAllWindows()