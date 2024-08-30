import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.common.by import By
from selenium.webdriver.common.action_chains import ActionChains
from selenium.webdriver.common.keys import Keys
from webdriver_manager.chrome import ChromeDriverManager
import time

def create_directory(directory_name):
    try:
        if not os.path.exists(directory_name):
            os.makedirs(directory_name)
            print(f"Directorio creado: {directory_name}")
        else:
            print(f"Directorio ya existe: {directory_name}")
    except OSError as e:
        print(f"Error al crear el directorio {directory_name}: {e}")

def download_images_from_url(url, directory_name):
    # Configuración de Selenium con ChromeDriver
    options = webdriver.ChromeOptions()
    options.add_argument('--headless')
    options.add_argument('--disable-gpu')
    options.add_argument('--no-sandbox')
    options.add_argument('--disable-dev-shm-usage')
    options.add_argument("user-agent=Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/58.0.3029.110 Safari/537.3")
    
    driver = webdriver.Chrome(service=Service(ChromeDriverManager().install()), options=options)
    
    # Navegar a la URL
    driver.get(url)
    time.sleep(3)  # Esperar a que se cargue la página completamente
    
    # Simular desplazamiento para cargar imágenes dinámicas
    last_height = driver.execute_script("return document.body.scrollHeight")
    
    while True:
        driver.execute_script("window.scrollTo(0, document.body.scrollHeight);")
        time.sleep(2)  # Esperar a que se cargue más contenido
        
        new_height = driver.execute_script("return document.body.scrollHeight")
        if new_height == last_height:
            break
        last_height = new_height
    
    # Obtener el contenido HTML
    soup = BeautifulSoup(driver.page_source, 'html.parser')
    
    # Buscar todas las etiquetas <img>
    img_tags = soup.find_all('img')

    # Descargar cada imagen encontrada
    for img in img_tags:
        img_url = img.get('src') or img.get('data-src') or img.get('srcset')
        if img_url:
            if ' ' in img_url:
                img_url = img_url.split(' ')[0]
            img_url = urljoin(url, img_url)
            img_name = os.path.basename(img_url)

            try:
                img_data = requests.get(img_url, headers={"User-Agent": "Mozilla/5.0"}).content
                with open(os.path.join(directory_name, img_name), 'wb') as f:
                    f.write(img_data)
                print(f"Imagen descargada: {img_name}")
            except Exception as e:
                print(f"Error descargando {img_name}: {e}")

    driver.quit()

if __name__ == "__main__":
    base_url = "https://www.freepik.es/fotos/manos-con-guantes/"
    uuid = "#uuid=0dbad0cd-6826-4062-900a-bbe74e4ff920"
    
    start_page = 6
    end_page = 10

    for page_number in range(start_page, end_page + 1):
        url = f"{base_url}{page_number}{uuid}"
        print(f"Procesando URL: {url}")
        
        directory_name = f"freepik_page_{page_number}"
        create_directory(directory_name)
        
        download_images_from_url(url, directory_name)
        time.sleep(2)
