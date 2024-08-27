#!/usr/bin/env python3

import os
from util import get_project, verify_env_var_presence


if __name__ == "__main__":
    env_list = [
        "CI_PIPELINE_SOURCE",
        "CI_PROJECT_NAME",
        "CI_PROJECT_NAMESPACE",
        "CI_JOB_ID",
    ]
    # pylint: disable=expression-not-assigned
    [verify_env_var_presence(e) for e in env_list]

    project = get_project()

    if os.environ["CI_PIPELINE_SOURCE"] != "merge_request_event":
        exit()  # no anouncement if  pipeline source is not a MR

    verify_env_var_presence("CI_MERGE_REQUEST_IID")

    merge_request = project.mergerequests.get(os.environ["CI_MERGE_REQUEST_IID"])

    docs_url = f"https://{os.environ['CI_PROJECT_NAMESPACE']}.gitlab.io/-/{os.environ['CI_PROJECT_NAME']}/-/jobs/{os.environ['DOCS_JOB_ID']}/artifacts/docs/html/index.html"
    merge_request.notes.create(
        {
            "body": f"The current docs can be viewed [here]({docs_url})",
            "created_at": merge_request.created_at,
        }
    )
