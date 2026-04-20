"""Unit tests for configure_logging."""

import logging

from src.app.log_setup import configure_logging
from src.app.startup_progress import NullProgressReporter, TextProgressReporter


def test_configure_logging_default_is_info():
    configure_logging(verbose=False, reporter=TextProgressReporter())
    assert logging.getLogger().level == logging.INFO


def test_configure_logging_verbose_is_debug():
    configure_logging(verbose=True, reporter=TextProgressReporter())
    assert logging.getLogger().level == logging.DEBUG


def test_configure_logging_replaces_existing_handlers():
    root = logging.getLogger()
    dummy = logging.Handler()
    root.addHandler(dummy)
    configure_logging(verbose=False, reporter=TextProgressReporter())
    assert dummy not in root.handlers


def test_configure_logging_text_reporter_uses_streamhandler():
    configure_logging(verbose=False, reporter=TextProgressReporter())
    root = logging.getLogger()
    assert len(root.handlers) == 1
    handler = root.handlers[0]
    assert isinstance(handler, logging.StreamHandler)


def test_configure_logging_null_reporter_falls_back_to_stream():
    configure_logging(verbose=False, reporter=NullProgressReporter())
    root = logging.getLogger()
    assert len(root.handlers) == 1
    assert isinstance(root.handlers[0], logging.StreamHandler)
