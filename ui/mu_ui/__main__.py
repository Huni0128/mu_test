"""Allow ``python -m ui.mu_ui`` for manual testing."""
from .app import _default_config, run_app


def main() -> None:  # pragma: no cover - convenience wrapper
    run_app(_default_config())


if __name__ == "__main__":  # pragma: no cover
    main()