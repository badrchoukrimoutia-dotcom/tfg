from semantic_action_plan import ( # importamos las piezas del sap que este modulo rellena
    build_sap,
    SemanticActionPlan,
    Constraint,
    ConstraintType,
    Preference,
    PreferenceType,
)

# este modulo no depende de ros 2: es traduccion pura de "lo que dijo el usuario" (nombres de habitacion)
# a "entidades ancladas al mapa" (acciones, restricciones y preferencias validas), facil de probar aislado


def ground(directives, room_data): # convierte las directivas del llm (destinos/restricciones/preferencias) en un sap valido
    valid_destinations, unknown_destinations = _validate_names(directives["destinations"], room_data)
    valid_constraint_targets, unknown_constraints = _validate_names(directives["constraints"], room_data)
    valid_preference_targets, unknown_preferences = _validate_names(directives["preferences"], room_data)

    # los destinos se anclan como acciones goto (la traduccion final a coordenadas la hace el sap_executor al ejecutarlas,
    # igual que ya se decidio en la fase 1); las restricciones/preferencias se anclan como zonas del mapa con su tipo
    plan = SemanticActionPlan(
        actions=build_sap(valid_destinations),
        constraints=[Constraint(constraint_type=ConstraintType.AVOID, target=name) for name in valid_constraint_targets],
        preferences=[Preference(preference_type=PreferenceType.SLOW, target=name) for name in valid_preference_targets],
    )

    unknown_names = { # habitaciones que el llm menciono pero que no existen en el mapa, para que el nodo las loguee
        "destinations": unknown_destinations,
        "constraints": unknown_constraints,
        "preferences": unknown_preferences,
    }

    return plan, unknown_names


def _validate_names(room_names, room_data): # separa los nombres que existen de verdad en el mapa de los que no
    valid = [] # nombres validos, ya en minusculas
    unknown = [] # nombres que el llm sugirio pero no estan en el mapa
    for room_name in room_names: # recorremos cada nombre sugerido
        room_name_lower = room_name.lower() # pasamos el nombre a minusculas para comparar
        if room_name_lower in room_data: # si la habitacion existe de verdad en el mapa
            valid.append(room_name_lower) # la anadimos a la lista de validos
        else: # si el llm se invento una habitacion que no existe
            unknown.append(room_name) # la anadimos a la lista de desconocidos, sin tocarla
    return valid, unknown
