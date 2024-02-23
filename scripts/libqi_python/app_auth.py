import qi
import sys


class Authenticator:

    def __init__(self, username, password):
        self.username = username
        self.password = password

    # This method is expected by libqi and must return a dictionary containing
    # login information with the keys 'user' and 'token'.
    def initialAuthData(self):
        return {"user": self.username, "token": self.password}


class AuthenticatorFactory:

    def __init__(self, username, password):
        self.username = username
        self.password = password

    # This method is expected by libqi and must return an object with at least
    # the `initialAuthData` method.
    def newAuthenticator(self):
        return Authenticator(self.username, self.password)


def make_application(program_path, qi_url, username="nao", password="nao"):
    """
    Create and return the qi.Application, with authentication set up
    """

    args = [program_path, "--qi-url", qi_url]
    app = qi.Application(args)
    factory = AuthenticatorFactory(username, password)
    app.session.setClientAuthenticatorFactory(factory)

    return app


if __name__ == "__main__":
    program_path = sys.argv[0]
    qi_url = "tcps://x.x.x.x:x"
    username = "nao"
    password = "nao"
    app = make_application(program_path, qi_url, username, password)
    app.start()
