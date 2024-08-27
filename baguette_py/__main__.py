from .baguette import Baguette


def main():
    instance = Baguette.getInstance()

    def dummy_setup():
        pass

    instance.start(dummy_setup)


if __name__ == "__main__":
    main()
