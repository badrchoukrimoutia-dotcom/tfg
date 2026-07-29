from dataclasses import dataclass # dataclass para definir la accion semantica de forma sencilla
from enum import Enum # enum para los tipos de accion y los estados posibles


class ActionType(Enum): # tipos de accion semantica que puede contener un sap
    GOTO = "GoTo" # por ahora solo navegar a una habitacion; se podran anadir mas tipos en el futuro


class ActionStatus(Enum): # estados posibles por los que pasa una accion del sap
    PENDING = "pending" # todavia no se ha empezado a ejecutar
    IN_PROGRESS = "in_progress" # el sap executor la esta ejecutando ahora mismo
    COMPLETED = "completed" # se ejecuto con exito
    FAILED = "failed" # no se pudo completar (rechazada por nav2, interrumpida sin retomar o fallo de nav2)


@dataclass
class SemanticAction: # una accion semantica de alto nivel dentro del sap (p. ej. goto una habitacion)
    action_type: ActionType # tipo de accion (goto, y en el futuro otros)
    target: str # objetivo semantico de la accion (nombre de la habitacion)
    status: ActionStatus = ActionStatus.PENDING # estado actual de la accion dentro del plan

    def to_dict(self): # representacion en json legible, para publicar y depurar el sap
        return {
            "action_type": self.action_type.value,
            "target": self.target,
            "status": self.status.value,
        }


def build_sap(room_names): # generacion del sap: construye la lista ordenada de acciones goto a partir de nombres ya validados
    return [SemanticAction(action_type=ActionType.GOTO, target=room_name) for room_name in room_names]
