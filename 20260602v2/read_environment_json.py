import json # importamos la libreria estandar para trabajar con archivos json

def load_environment_json(json_path): # funcion que abre el json del entorno y devuelve todo su contenido
    try: # intentamos abrir y leer el archivo de forma segura
        with open(json_path, 'r', encoding='utf-8') as f: # abrimos el archivo en modo lectura con codificacion utf-8
            return json.load(f) # devolvemos el diccionario completo (habitaciones y conexiones)
    except Exception as e: # si algo falla durante la lectura capturamos el error
        print(f"Error reading the environment JSON: {e}") # avisamos por terminal de cual ha sido el problema
        return None # devolvemos None para indicar que no se ha podido cargar nada
