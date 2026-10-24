# API reference

Every public class, method and field in `org.horizon36596.simloop`, with **units and frame stated on
every number**.

<a href="../javadoc/index.html" class="md-button md-button--primary">Open the API reference</a>

That link opens the generated Javadoc beside this guide, both on the published site and in a full
local build. Build the guide and API reference together from the repository root:

```powershell
./gradlew :SimLoop:docsSite
```

Preview the complete build from the repository root:

```powershell
python -m http.server 8000 --bind 127.0.0.1 --directory SimLoop/build/docs/site
```

Open [the local API landing page](http://127.0.0.1:8000/api-reference/). Stop the server with `Ctrl+C`.
Running `mkdocs serve` alone previews the guide but does not copy the API pages. For Javadoc alone,
run `:SimLoop:apiDocs` and open `SimLoop/build/docs/apiDocs/index.html`.

(Run it from the repository root, not from inside `SimLoop/`. The library is a Gradle subproject of a
repository that holds only it, so the `:SimLoop` prefix is correct.)

## How it is kept complete

Javadoc is generated with `-Xdoclint:all` and `-Werror`. Missing comments, parameters, return
descriptions and broken `{@link}` references fail the build. Units and coordinate frames belong in
the public API comments so callers do not have to infer them from a method name.

The release's Javadoc jar is built from the same generated reference as that source checkout. Use
the jar for your installed tag when you need a matching API reference: this unversioned site can
describe development APIs newer than the published dependency.
