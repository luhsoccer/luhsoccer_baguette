import baguette_py as baguette
import start_config


@start_config.event_based(start_config.setup_clean)
def test_skills(
    baguette_instance: baguette.Baguette, skill_library: baguette.SkillLibrary
) -> None:
    print(skill_library.getSkill(baguette.GameSkillNames.GoToPoint))
