const fs = require('fs')
const util = require('util')
const promisify = util.promisify
const exec = promisify(require('child_process').exec);
const readFile = promisify(fs.readFile)
const symlink = promisify(fs.symlink)
const unlink = promisify(fs.unlink)

async function trimmedStdout(command) {
  return (await exec(command)).stdout.trim()
}

async function getBoardNames() {
  const content = await readFile("sam/boards.txt", 'utf8');
  const lines = content.split('\n');
  return lines
    .map(line => /^.*\.name=(.*)$/.exec(line))
    .filter(match => match !== null)
    .map(match => match[1]);
}

async function main() {
  const userOrOrg = 'macchina'
  const repositoryName = 'arduino-boards-sam'
  
  //await unlink('macchina')
  await symlink('sam', 'macchina')
  const version = await trimmedStdout('git describe');

  const filename = `macchina-sam-${version}.tar.gz`
  await exec(`tar -h -cvzf ${filename} macchina/ --exclude=".*"`)
  const hash = await trimmedStdout(`sha256sum ${filename} | sed 's/\\s.*//'`)
  const byteCount = await trimmedStdout(`du -b ${filename} | sed 's/\\s.*//'`)
  const boardNames = await getBoardNames()

  const output = {
    name: 'Macchina SAM Boards (Install "Arduino SAM Boards" first)',
    architecture: "sam",
    version: version,
    category: "Contributed",
    url: `https://github.com/${userOrOrg}/${repositoryName}/releases/download/${version}/${filename}`,
    archiveFileName: filename,
    checksum: `SHA-256:${hash}`,
    size: byteCount,
    boards: boardNames.map(name => { return { name } }),
    toolsDependencies: [
      {
        packager: "arduino",
        name: "arm-none-eabi-gcc",
        version: "4.8.3-2014q1"
      },
      {
        packager: "arduino",
        name: "bossac",
        version: "1.6.1-arduino"
      }
    ]
  }

  return JSON.stringify(output, undefined, 2);
}


if (require.main === module) {
  main(process.argv)
    .then(s => console.log(s))
    .catch(error => {
      console.error(error);
      process.exit(2);
    });
}
