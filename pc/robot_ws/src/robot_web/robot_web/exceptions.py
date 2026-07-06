class GatewayError(Exception):
    """Public API error with an HTTP-oriented status code."""

    def __init__(self, status_code, reason, message, detail=None):
        super().__init__(message)
        self.status_code = int(status_code)
        self.reason = str(reason)
        self.message = str(message)
        self.detail = detail or {}

    def to_dict(self):
        return {
            "ok": False,
            "reason": self.reason,
            "message": self.message,
            "detail": self.detail,
        }
