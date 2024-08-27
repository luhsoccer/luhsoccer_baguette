#!/usr/bin/env python3
import os
import re
import sys
import semver
import subprocess
import gitlab


def git(*args):
    return subprocess.check_output(["git"] + list(args))


def verify_env_var_presence(name):
    if name not in os.environ:
        raise Exception(
            f"Expected the following environment variable to be set: {name}"
        )


def extract_gitlab_url_from_project_url():
    project_url = os.environ["CI_PROJECT_URL"]
    project_path = os.environ["CI_PROJECT_PATH"]

    return project_url.split(f"/{project_path}", 1)[0]


def extract_merge_request_id_from_commit():
    message = git("log", "-1", "--pretty=%B")
    matches = re.search(r"(\S*\/\S*!)(\d+)", message.decode("utf-8"), re.M | re.I)

    if matches == None:
        raise Exception(
            f"Unable to extract merge request from commit message: {message}"
        )

    return matches.group(2)


def retrieve_labels_from_merge_request(merge_request_id):
    project_id = os.environ["CI_PROJECT_ID"]
    gitlab_private_token = os.environ["BOT_TOKEN"]

    gl = gitlab.Gitlab(
        extract_gitlab_url_from_project_url(), private_token=gitlab_private_token
    )
    gl.auth()

    project = gl.projects.get(project_id)
    merge_request = project.mergerequests.get(merge_request_id)

    return merge_request.labels


def bump(latest):
    minor_bump_label = os.environ.get("MINOR_BUMP_LABEL") or "bump-minor"
    major_bump_label = os.environ.get("MAJOR_BUMP_LABEL") or "bump-major"

    merge_request_id = extract_merge_request_id_from_commit()
    labels = retrieve_labels_from_merge_request(merge_request_id)

    if minor_bump_label in labels:
        return semver.bump_minor(latest)
    elif major_bump_label in labels:
        return semver.bump_major(latest)
    else:
        return semver.bump_patch(latest)


def tag_repo(tag):
    repository_url = os.environ["CI_REPOSITORY_URL"]
    username = os.environ["BOT_USERNAME"]
    password = os.environ["BOT_TOKEN"]

    push_url = re.sub(
        r"([a-z]+://)[^@]*(@.*)", rf"\g<1>{username}:{password}\g<2>", repository_url
    )
    tag = "v" + tag
    git("fetch", "origin", "refs/tags/*:refs/tags/*", "--prune")
    git("remote", "set-url", "--push", "origin", push_url)
    git("tag", tag)
    git("push", "origin", tag)


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "tag":
        # develop
        # Dev tagging is disabled since it would result in too many tags
        # tag_repo(sys.argv[2])
        pass
    else:
        # main
        env_list = [
            "CI_REPOSITORY_URL",
            "CI_PROJECT_ID",
            "CI_PROJECT_URL",
            "CI_PROJECT_PATH",
            "BOT_USERNAME",
            "BOT_TOKEN",
            "CI_PROJECT_ID",
        ]
        [verify_env_var_presence(e) for e in env_list]

        try:
            latest = (
                git(
                    "describe",
                    "--tags",
                    "--first-parent",
                    "--match",
                    "v[[:digit:]]*.[[:digit:]]*.[[:digit:]]*",
                )
                .decode()
                .strip()
            )
        except subprocess.CalledProcessError:
            # Default to version 1.0.0 if no tags are available
            version = "1.0.0"
        else:
            # Skip already tagged commits
            if "-" not in latest:
                print(latest)
                return 0

            version = bump(latest[1:])

        tag_repo(version)
        print(version, flush=True)

        # check diff of main and develop

        git("fetch")
        git("checkout", "develop")
        git("fetch", "origin", "main:main")
        diff = git("diff", "main")
        print(diff, flush=True)
        if len(diff) == 0:
            # merge main back to develop if no diff
            print("No diff", flush=True)
            git("config", "user.name", os.environ["BOT_USERNAME"])
            git("config", "user.email", "software@luhbots.de")
            git("config", "-l")
            git("merge", "main")
            git("push")
        else:
            # create MR otherwise, so hotfixes can be merged back
            gitlab_private_token = os.environ["BOT_TOKEN"]
            gl = gitlab.Gitlab(
                extract_gitlab_url_from_project_url(),
                private_token=gitlab_private_token,
            )
            gl.auth()
            project = gl.projects.get(os.environ["CI_PROJECT_ID"])

            # check if mr main to develop already exists
            mrs = project.mergerequests.list(
                state="opened", source_branch="main", target_branch="develop"
            )
            if len(mrs) == 0:
                mr = project.mergerequests.create(
                    {
                        "source_branch": "main",
                        "target_branch": "develop",
                        "title": f"Merge v{version} back to develop branch",
                        "squash": False,
                    }
                )

        with open("./baguette_version", "w", encoding="UTF-8") as file:
            file.write(version)
    return 0


if __name__ == "__main__":
    sys.exit(main())
