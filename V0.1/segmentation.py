import cv2
import numpy as np

def preProcessing(imgf, a, b):
    img = cv2.convertScaleAbs(imgf, alpha=a, beta=b)
    cv2.imshow("Algo", img)
    img = np.where(img <= 40, 0, img)
    img = np.where(img > 40, 255, img)
    gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    cv2.imshow("Algo2", gray_img)
    cts, _ = cv2.findContours(gray_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_cnr = max(cts, key=cv2.contourArea)
    e = 0.01 * cv2.arcLength(largest_cnr, True)
    apx = cv2.approxPolyDP(largest_cnr, e, True)
    return apx

def getVortex(apx):
    vortex = []
    xif = []
    yif = []
    for pointf in apx:
        xf, yf = pointf[0]
        vortex.append((xf, yf))
    for vort in vortex:
        xif.append(vort[0])
        yif.append(vort[1])
    return xif, yif

def calcParameters1(xif, yif):
    mf = []
    bf = []
    for i in range(0, 7, 2):
        m_linef = (yif[i+1] - yif[i]) / (xif[i+1] - xif[i])
        mf.append(m_linef)
        b_linef = yif[i] - m_linef * xif[i]
        bf.append(b_linef)
    return mf, bf

def calcParameters2(xif, yif):
    mf = []
    bf = []
    for i in range(1, 6, 2):
        m_linef = (yif[i+1] - yif[i]) / (xif[i+1] - xif[i])
        mf.append(m_linef)
        b_linef = yif[i] - m_linef * xif[i]
        bf.append(b_linef)
        if i==5:
            i = 7
            m_linef = (yif[0] - yif[i]) / (xif[0] - xif[i])
            mf.append(m_linef)
            b_linef = yif[i] - m_linef * xif[i]
            bf.append(b_linef)
    return mf, bf

def extendLine(xif, yif, mf, imgf, bf):
    for i in range(len(mf)-1):
        #Puntos extremos originales de la línea
        x1 = xif[i]
        y1 = yif[i]
        x2 = xif[i+1]
        y2 = yif[i+1]

        #Calcular puntos adicionales para extender la línea
        ext_length = 1000  # Longitud de la extensión, puedes ajustar esto según sea necesario
        x1_ext = int(x1 - ext_length)
        y1_ext = int(mf[i] * x1_ext + bf[i])
        x2_ext = int(x2 + ext_length)
        y2_ext = int(mf[i] * x2_ext + bf[i])

        #Dibujar la línea extendida
        cv2.line(imgf, (x1_ext, y1_ext), (x2_ext, y2_ext), (0, 0, 0), 5)

    #Línea con coordenadas, ni[3], ni[0]
    x1 = xif[0] 
    x2 = xif[3]
    y1 = yif[0]
    y2 = yif[3]
    ext_length = 1000
    x1_ext = int(x1 - ext_length)
    y1_ext = int(mf[3] * x1_ext + b[3])
    x2_ext = int(x2 + ext_length)
    y2_ext = int(mf[3] * x2_ext + b[3]) 

    #Dibujar la línea especial extendida
    cv2.line(imgf, (x1_ext, y1_ext), (x2_ext, y2_ext), (0, 0, 0), 5)

def intersection(m1, b1, m2, b2):
    if m1 == m2:
        return None  # Las líneas son paralelas, no hay intersección
    xf = (b2 - b1) / (m1 - m2)
    yf = m1 * xf + b1
    return int(xf), int(yf)


    for i in range(len(mf)-1):
        #Puntos extremos originales de la línea
        x1 = xif[i]
        y1 = yif[i]
        x2 = xif[i+1]
        y2 = yif[i+1]

        #Calcular puntos adicionales para extender la línea
        ext_length = 1000  # Longitud de la extensión, puedes ajustar esto según sea necesario
        x1_ext = int(x1 - ext_length)
        y1_ext = int(mf[i] * x1_ext + bf[i])
        x2_ext = int(x2 + ext_length)
        y2_ext = int(mf[i] * x2_ext + bf[i])

        #Dibujar la línea extendida
        cv2.line(imgf, (x1_ext, y1_ext), (x2_ext, y2_ext), (0, 0, 0), 5)

    #Línea con coordenadas, ni[3], ni[0]
    x1 = xif[0] 
    x2 = xif[3]
    y1 = yif[0]
    y2 = yif[3]
    ext_length = 1000
    x1_ext = int(x1 - ext_length)
    y1_ext = int(m[3] * x1_ext + b[3])
    x2_ext = int(x2 + ext_length)
    y2_ext = int(m[3] * x2_ext + b[3])

    #Dibujar la línea especial extendida
    cv2.line(imgf, (x1_ext, y1_ext), (x2_ext, y2_ext), (0, 0, 0), 5)

def mapValue(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

def calculateCenter(image, center_x, center_y, red, green, blue):
    cv2.circle(image, (center_x, center_y), 5, (blue, green, red), -1)  

maxWidth = 1020
maxHeight = 720
image = cv2.imread('mega.jpg')
image = cv2.resize(image, (maxWidth, maxHeight))
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#Parámetros iniciales
alpha = 1    #Contraste
beta = -180  #Brillo inicial (cambia con el pre procesamiento)

#Pre procesamiento de la imagen
approx = preProcessing(image, alpha, beta)

#Encontramos el número de vertices por fuerza bruta
while len(approx) != 8:
    beta -= 1
    approx = preProcessing(image, alpha, beta)

#Listas que almacenan las coordenadas de cada vertice
xi, yi = getVortex(approx)

#Calcula la pendiente y la intersección con 'y'
m, b = calcParameters1(xi, yi)

#Encuentra las intersecciones con cada recta
ix1, iy1 = intersection(m[0], b[0], m[1], b[1])
ix2, iy2 = intersection(m[1], b[1], m[2], b[2])
ix3, iy3 = intersection(m[2], b[2], m[3], b[3])
ix4, iy4 = intersection(m[3], b[3], m[0], b[0])

#Manipula el cálculo de las rectas para arreglar la orientación
if(ix1 > 0 and ix1 < maxWidth and iy1 > 0 and iy1 < maxHeight
and ix2 > 0 and ix2 < maxWidth and iy2 > 0 and iy2 < maxHeight
and ix3 > 0 and ix3 < maxWidth and iy3 > 0 and iy3 < maxHeight
and ix4 > 0 and ix4 < maxWidth and iy4 > 0 and iy4 < maxHeight
):
    pass
else:
    cdr = 0
    while(ix1 < 0 or ix1 > maxWidth or iy1 < 0 or iy1 > maxHeight
        or ix2 < 0 or ix2 > maxWidth or iy2 < 0 or iy2 > maxHeight
        or ix3 < 0 or ix3 > maxWidth or iy3 < 0 or iy3 > maxHeight
        or ix4 < 0 or ix4 > maxWidth or iy4 < 0 or iy4 > maxHeight
        ):
        if cdr == 0:
            m, b = calcParameters1(xi, yi)
            ix1, iy1 = intersection(m[0], b[0], m[1], b[1])
            ix2, iy2 = intersection(m[1], b[1], m[2], b[2])
            ix3, iy3 = intersection(m[2], b[2], m[3], b[3])
            ix4, iy4 = intersection(m[3], b[3], m[0], b[0])
        elif cdr == 1:
            m, b = calcParameters2(xi, yi)
            ix1, iy1 = intersection(m[0], b[0], m[1], b[1])
            ix2, iy2 = intersection(m[1], b[1], m[2], b[2])
            ix3, iy3 = intersection(m[2], b[2], m[3], b[3])
            ix4, iy4 = intersection(m[3], b[3], m[0], b[0])
        cdr += 1

#Dibujamos las líneas de segmentación
extendLine(xi, yi, m, image, b)

#Imprimimos los puntos para debuggear
print(f"x1, y1: {ix1, iy1}")
print(f"x2, y2: {ix2, iy2}")
print(f"x3, y3: {ix3, iy3}")
print(f"x4, y4: {ix4, iy4}")

#Dibujamos los puntos
cv2.circle(image, (ix1, iy1), 1, (0,117,255), 20)
cv2.circle(image, (ix2, iy2), 1, (0,117,255), 20)
cv2.circle(image, (ix3, iy3), 1, (0,117,255), 20)
cv2.circle(image, (ix4, iy4), 1, (0,117,255), 20)

cv2.imshow("Resultados", image)
cv2.waitKey(0)
cv2.destroyAllWindows()