from dataclasses import dataclass, field # dataclass para definir las entidades semanticas de forma sencilla
from enum import Enum # enum para los tipos de accion, restriccion, preferencia y los estados posibles


class ActionType(Enum): # tipos de accion semantica que puede contener un sap
    GOTO = "GoTo" # por ahora solo navegar a una habitacion; se podran anadir mas tipos en el futuro


class ActionStatus(Enum): # estados posibles por los que pasa una accion del sap
    PENDING = "pending" # todavia no se ha empezado a ejecutar
    IN_PROGRESS = "in_progress" # el sap executor la esta ejecutando ahora mismo
    COMPLETED = "completed" # se ejecuto con exito
    FAILED = "failed" # no se pudo completar (rechazada por nav2, interrumpida sin retomar o fallo de nav2)


class ConstraintType(Enum): # tipos de restriccion que puede contener un sap
    AVOID = "avoid" # zona que debe evitarse por completo; se podran anadir mas tipos en el futuro


class PreferenceType(Enum): # tipos de preferencia que puede contener un sap
    SLOW = "slow" # ir despacio por la zona; se podran anadir mas tipos en el futuro


@dataclass
class SemanticAction: # una accion semantica de alto nivel dentro del sap (p. ej. goto una habitacion)
    action_type: ActionType # tipo de accion (goto, y en el futuro otros)
    target: str # objetivo semantico de la accion (nombre de la habitacion)
    status: ActionStatus = ActionStatus.PENDING # estado actual de la accion dentro del plan
    retry_count: int = 0 # cuantas veces se ha reintentado esta accion tras un fallo (fase 3: feedback y replanteamiento)

    def to_dict(self): # representacion en json legible, para publicar y depurar el sap
        return {
            "action_type": self.action_type.value,
            "target": self.target,
            "status": self.status.value,
            "retry_count": self.retry_count,
        }


@dataclass
class Constraint: # una restriccion semantica: una zona que debe evitarse
    constraint_type: ConstraintType # tipo de restriccion (avoid, y en el futuro otros)
    target: str # zona/habitacion a la que se aplica la restriccion

    def to_dict(self): # representacion en json legible, para publicar y depurar las restricciones activas
        return {
            "constraint_type": self.constraint_type.value,
            "target": self.target,
        }


@dataclass
class Preference: # una preferencia semantica: una zona con un comportamiento asociado (p. ej. ir despacio)
    preference_type: PreferenceType # tipo de preferencia (slow, y en el futuro otros)
    target: str # zona/habitacion a la que se aplica la preferencia

    def to_dict(self): # representacion en json legible, para publicar y depurar las preferencias activas
        return {
            "preference_type": self.preference_type.value,
            "target": self.target,
        }


@dataclass
class SemanticActionPlan: # el sap completo: acciones a ejecutar en orden, mas restricciones y preferencias activas
    actions: list[SemanticAction] = field(default_factory=list) # cola ordenada de acciones goto (lo que ejecuta el sap_executor)
    constraints: list[Constraint] = field(default_factory=list) # zonas a evitar detectadas en la orden
    preferences: list[Preference] = field(default_factory=list) # zonas con preferencia asociada detectadas en la orden


def build_sap(room_names): # generacion de la cola de acciones: construye la lista ordenada de acciones goto a partir de nombres ya validados
    return [SemanticAction(action_type=ActionType.GOTO, target=room_name) for room_name in room_names]
