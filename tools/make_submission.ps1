<#
.SYNOPSIS
  Builds the CAS source-code zip: 2627_CAS_source_<matr1>_<matr2>_<matr3>.zip

.DESCRIPTION
  Zips firmware/, tools/, docs/, README.md and CHANGELOG.md from the repository root into
  L4_Rover/06_REPORTS_AND_DEMO/Final_Submission/ (outside the repo). Git metadata, virtual
  environments, build output and PNG plots are left out. Matriculation numbers come from
  -Matriculation or, if omitted, one per line from Final_Submission/matriculation.txt.

.EXAMPLE
  .\tools\make_submission.ps1 -Matriculation 1234567,2345678,3456789
#>
param(
  [string[]]$Matriculation
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
# L4_Rover/03_ENGINEERING/Rover-l4-automation -> L4_Rover/06_REPORTS_AND_DEMO/Final_Submission
$subDir = Join-Path (Split-Path -Parent (Split-Path -Parent $root)) "06_REPORTS_AND_DEMO\Final_Submission"
New-Item -ItemType Directory -Path $subDir -Force | Out-Null

if (-not $Matriculation -or $Matriculation.Count -eq 0) {
  $file = Join-Path $subDir "matriculation.txt"
  if (-not (Test-Path $file)) {
    throw "Pass -Matriculation n1,n2,n3 or create $subDir\matriculation.txt (one number per line)."
  }
  $Matriculation = Get-Content $file | Where-Object { $_.Trim() -ne "" } | ForEach-Object { $_.Trim() }
}
foreach ($m in $Matriculation) {
  if ($m -notmatch '^\d+$') { throw "Matriculation number '$m' is not numeric." }
}

$zipName = "2627_CAS_source_" + ($Matriculation -join "_") + ".zip"
$zipPath = Join-Path $subDir $zipName

$staging = Join-Path $env:TEMP ("cas_submission_" + [guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $staging | Out-Null
try {
  $include = @("firmware", "tools", "docs", "README.md", "CHANGELOG.md")
  foreach ($item in $include) {
    $src = Join-Path $root $item
    if (Test-Path $src) { Copy-Item $src -Destination (Join-Path $staging $item) -Recurse }
  }
  # strip what must not be submitted
  Get-ChildItem $staging -Recurse -Force -Include ".venv", "venv", "__pycache__", "build", ".arduino-build" -Directory |
    Remove-Item -Recurse -Force
  Get-ChildItem $staging -Recurse -Force -Include "*.png", "*.pyc", "*.local.*" -File |
    Remove-Item -Force

  if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
  Compress-Archive -Path (Join-Path $staging "*") -DestinationPath $zipPath -CompressionLevel Optimal
  $size = [math]::Round((Get-Item $zipPath).Length / 1KB)
  Write-Host "Written $zipPath ($size KB)"
  Write-Host "Check: git status clean? CHANGELOG entry dated? tag set?"
}
finally {
  Remove-Item $staging -Recurse -Force -ErrorAction SilentlyContinue
}
