from dataclasses import dataclass # dataclass para representar el resultado de una accion de forma sencilla
from enum import Enum # enum para los motivos de fallo posibles


class FailureReason(Enum): # motivos por los que una accion del sap puede fallar
    REJECTED_BY_NAV2 = "rejected_by_nav2" # nav2 rechazo la meta nada mas enviarla
    NOT_REACHED = "not_reached" # nav2 acepto la meta pero no la pudo completar (abortada o cancelada)


@dataclass
class ActionOutcome: # resultado de ejecutar una accion del sap, con el motivo si fallo
    action: object # la accion del sap (SemanticAction); no se tipa aqui para no acoplar este modulo al sap
    succeeded: bool # si la accion se completo con exito
    reason: FailureReason = None # motivo del fallo, o none si tuvo exito

    def to_dict(self): # representacion en json legible, para publicar y depurar el bucle de feedback
        return {
            "target": self.action.target,
            "succeeded": self.succeeded,
            "reason": self.reason.value if self.reason is not None else None,
        }


class ExecutionFeedbackManager: # centraliza la monitorizacion de la ejecucion: registra el resultado de cada accion del sap
    def __init__(self):
        self.outcomes = [] # historial de resultados registrados, por si alguna politica futura quiere consultarlo

    def record_success(self, action): # registra que una accion se completo con exito
        outcome = ActionOutcome(action=action, succeeded=True)
        self.outcomes.append(outcome)
        return outcome

    def record_failure(self, action, reason): # registra que una accion ha fallado, junto con su motivo
        outcome = ActionOutcome(action=action, succeeded=False, reason=reason)
        self.outcomes.append(outcome)
        return outcome
