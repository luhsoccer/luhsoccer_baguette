#!/usr/bin/env python3

import os
from util import get_project, verify_env_var_presence


class MissingLabel(Exception):
    def __init__(self, missing_label_category):
        super().__init__(f"Merge request misses a '{missing_label_category}' label ")


if __name__ == "__main__":
    env_list = [
        "CI_PIPELINE_SOURCE",
    ]
    # pylint: disable=expression-not-assigned
    [verify_env_var_presence(e) for e in env_list]

    project = get_project()

    if os.environ["CI_PIPELINE_SOURCE"] != "merge_request_event":
        exit()  # no checks if pipeline source is not a MR

    if os.environ["CI_MERGE_REQUEST_EVENT_TYPE"] == "merge_train":
        exit()  # no checks if pipeline source is a merge train. The check already happened in the original MR

    verify_env_var_presence("CI_MERGE_REQUEST_ID")

    merge_request = project.mergerequests.get(os.environ["CI_MERGE_REQUEST_IID"])

    automatic_merge = merge_request.merge_when_pipeline_succeeds

    if automatic_merge:
        merge_request.cancel_merge_when_pipeline_succeeds()

    required_label_categories = {"Module", "Prio", "Type"}

    # print(merge_request.labels)
    type_label = ""
    module_label = ""
    # check label
    for required_label in required_label_categories:
        found = False
        for label in merge_request.labels:
            if required_label + "::" in label:
                found = True
                if required_label == "Type":
                    type_label = label
                if required_label == "Module":
                    module_label = label
                break

        if not found:
            raise MissingLabel(required_label)

    # check mr title and correct

    title: str = merge_request.title
    is_draft: bool = title.startswith("Draft:")
    title = title.replace("Draft:", "")
    words = title.split()  # Replace draft label so it doesn't interfere with the check
    if len(words) < 2 or not words[0].endswith(":"):
        # title has no conventional commit specifier, add depending on label

        TYPE_LABEL_TO_CONV_COMMIT: dict[str, str] = {
            "Bug": "fix",
            "Build": "build",
            "CI": "ci",
            "Chore": "chore",
            "Docs": "docs",
            "Feature": "feat",
            "Performance": "perf",
            "Refactor": "refactor",
            "Revert": "revert",
            "Style": "style",
            "Test": "test",
        }
        category = type_label.split("::")[1]
        if category in TYPE_LABEL_TO_CONV_COMMIT:
            prefix = ""
            if is_draft:
                prefix = "Draft: "
            merge_request.title = f"{prefix} {TYPE_LABEL_TO_CONV_COMMIT[category]}({module_label.split('::')[1]}): {title}"

            replace = False
            for line in merge_request.description.splitlines():
                if line.startswith("Changelog: "):
                    merge_request.description = merge_request.description.replace(
                        line, f"Changelog: {TYPE_LABEL_TO_CONV_COMMIT[category]}"
                    )
                    replace = True
                    break

            if not replace:
                merge_request.description = (
                    merge_request.description
                    + "\n"
                    + f"Changelog: {TYPE_LABEL_TO_CONV_COMMIT[category]}"
                )

            merge_request.save()

    if automatic_merge:
        merge_request.merge(merge_when_pipeline_succeeds=True)
