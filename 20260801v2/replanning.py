from enum import Enum # enum para las decisiones posibles del modulo de replanteamiento


class ReplanningDecision(Enum): # decisiones que el modulo de replanteamiento puede tomar ante el fallo de una accion
    RETRY = "retry" # reintentar la misma accion tras un breve retraso
    GIVE_UP = "give_up" # marcarla como fallida definitivamente y continuar con la siguiente del plan
    # disenado para admitir mas politicas en el futuro (p. ej. ASK_USER, ABORT_PLAN): basta con anadir
    # el valor aqui y contemplarlo en decide() y en quien la aplique (sap_executor.py)


class ReplanningPolicy: # decide que politica aplicar ante el fallo de una accion del sap
    def __init__(self, max_retries=2, retry_delay_seconds=2.0): # numero maximo de reintentos y espera entre uno y otro
        self.max_retries = max_retries # reintentos antes de rendirse (por defecto 2 reintentos = 3 intentos en total)
        self.retry_delay_seconds = retry_delay_seconds # espera antes de reintentar, para dar margen a que la causa del fallo desaparezca

    def decide(self, outcome): # decide reintentar o rendirse, segun cuantas veces ya se ha reintentado esta accion
        if outcome.action.retry_count < self.max_retries: # si todavia quedan reintentos disponibles
            return ReplanningDecision.RETRY
        return ReplanningDecision.GIVE_UP # se agotaron los reintentos: nos rendimos con esta accion
