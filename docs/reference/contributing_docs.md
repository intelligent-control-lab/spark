# Contributing documentation

Documentation source lives beside the code in the `docs/` directory. Update
the relevant page in the same pull request as a user-visible behavior change.

## Preview locally

Create a lightweight documentation environment:

```bash
python -m venv .venv-docs
source .venv-docs/bin/activate
python -m pip install -r docs/requirements.txt
make -C docs livehtml
```

Open <http://127.0.0.1:8000> and edit Markdown files. The browser refreshes
when a source file changes.

## Validate

```bash
make -C docs clean html
```

The build treats warnings as errors. New pages must be reachable from a
`toctree`, internal links must resolve, and code examples should use commands
that exist in the current repository.

## Publish locally

With `spark` and `spark-docs-site` checked out next to one another:

```bash
./tools/publish_docs.sh
git -C ../spark-docs-site status
```

The script builds the site and synchronizes generated HTML into the publishing
repository. Review those changes before committing or pushing them.

The GitHub workflow always uploads the generated site as an artifact. Remote
publication is optional: configure the repository variable
`DOCS_PUBLISH_REPOSITORY` (for example,
`intelligent-control-lab/spark-docs-site`) and the write-capable secret
`DOCS_PUBLISH_TOKEN` only after a publishing repository is available.
