# NEXUS iOS cloud build

Prepared for a dedicated GitHub repository. Manual macos-15 workflow, 15 minute limit, read-only repository permission, no credentials persisted. It deliberately skips private repositories to avoid assuming paid Actions usage. Do not upload Android keys, DPAPI files, accounts or device logs. This folder only contains iOS preparation source and a workflow.

Once the owner approves publishing these sources to a dedicated public repository, the workflow can be run. No cloud job has been started yet. No IPA is produced: the package has library targets only and the SoccerStars integration is absent. Apple signing is not required for the simulator library compile; it is still required for a distributable device app.

Official cloud host details: https://docs.github.com/en/actions/reference/runners/github-hosted-runners
