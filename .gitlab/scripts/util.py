#!/usr/bin/env python3
import os
import gitlab

MAIN_BRANCH_NAME = "main"
DEV_BRANCH_NAME = "develop"


def extract_gitlab_url_from_project_url():
    project_url = os.environ["CI_PROJECT_URL"]
    project_path = os.environ["CI_PROJECT_PATH"]

    return project_url.split(f"/{project_path}", 1)[0]


def verify_env_var_presence(name):
    if name not in os.environ:
        raise Exception(
            f"Expected the following environment variable to be set: {name}"
        )


def get_project():
    # check if environment variables are set
    env_list = ["CI_PROJECT_ID", "BOT_TOKEN", "CI_PROJECT_URL", "CI_PROJECT_PATH"]
    [verify_env_var_presence(e) for e in env_list]

    project_id = os.environ["CI_PROJECT_ID"]
    gitlab_private_token = os.environ["BOT_TOKEN"]

    gl = gitlab.Gitlab(
        extract_gitlab_url_from_project_url(), private_token=gitlab_private_token
    )
    gl.auth()

    return gl.projects.get(project_id)
