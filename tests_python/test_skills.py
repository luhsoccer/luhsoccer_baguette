from tests_python.conftest import skill_book, baguette
import pytest


def test_skills(skill_book: baguette.SkillBook) -> None:
    print(skill_book.getSkill(baguette.SkillName.GoToPoint))
