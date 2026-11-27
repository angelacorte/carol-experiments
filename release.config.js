var prepareCmd = `
echo version=\${nextRelease.version} > gradle.properties
echo VERSION="\${nextRelease.version}" > .env
echo PROJECT_NAME=$(grep -Po 'rootProject\\s*\\.\\s*name\\s*=\\s*"\\K[\\w-]+(?=")' settings.gradle.kts) >> .env
`
// docker compose build
// docker compose push
// Maven Central publication of :core, disabled for now. To enable, put this line back as the first line of publishCmd
// (it must run first, while HEAD is still the release tag, so gitSemVer yields a clean version):
// ./gradlew :core:publishAllPublicationsToProjectLocalRepository :core:zipMavenCentralPortalPublication :core:releaseMavenCentralPortalPublication || exit 1
var publishCmd = `
git add gradle.properties .env
git commit -m "chore(release): update gradle.properties .env versions to \${nextRelease.version} [skip ci]"
git push
`
var config = require('semantic-release-preconfigured-conventional-commits');
config.plugins.push(
    ["@semantic-release/exec", {
        "prepareCmd": prepareCmd,
        "publishCmd": publishCmd,
    }],
    "@semantic-release/github",
    "@semantic-release/git",
)
module.exports = config
