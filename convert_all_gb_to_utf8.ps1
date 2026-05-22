$projectRoot = "c:\Users\souph\Documents\CCSProjects\ACAC\AC_AC Buck-Boost"
$extensions = @("*.c", "*.h", "*.asm", "*.cmd", "*.gel")
$convertedCount = 0
$skippedCount = 0
$errorCount = 0

Write-Host "Scanning for GB-encoded files in: $projectRoot"
Write-Host "================================================"

foreach ($ext in $extensions) {
    $files = Get-ChildItem -Path $projectRoot -Recurse -Filter $ext
    foreach ($file in $files) {
        $path = $file.FullName
        try {
            $bytes = [System.IO.File]::ReadAllBytes($path)
            
            # Skip empty files
            if ($bytes.Length -eq 0) {
                $skippedCount++
                continue
            }
            
            # Check if file has any non-ASCII bytes (> 0x7F)
            $hasNonAscii = $false
            foreach ($b in $bytes) {
                if ($b -gt 0x7F) {
                    $hasNonAscii = $true
                    break
                }
            }
            
            if (-not $hasNonAscii) {
                # Pure ASCII file - no conversion needed
                $skippedCount++
                continue
            }
            
            # Check if file already has UTF-8 BOM (EF BB BF)
            if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
                Write-Host "  [SKIP] $($file.Name) - already has UTF-8 BOM"
                $skippedCount++
                continue
            }
            
            # Try to decode as UTF-8 and check for replacement characters
            $utf8Text = [System.Text.Encoding]::UTF8.GetString($bytes)
            $hasReplacementChar = $utf8Text.Contains([System.Char]::ConvertFromUtf32(0xFFFD))
            
            if (-not $hasReplacementChar) {
                # Also decode as GB2312 and compare
                $gbText = [System.Text.Encoding]::GetEncoding(936).GetString($bytes)
                if ($utf8Text -eq $gbText) {
                    # Both decodings produce same result - it's ASCII-compatible, no conversion needed
                    $skippedCount++
                    continue
                }
                # Check if UTF-8 text has meaningful Chinese characters already
                # If both decode differently but neither has replacement chars,
                # prefer UTF-8 (already valid UTF-8)
                Write-Host "  [SKIP] $($file.Name) - already valid UTF-8"
                $skippedCount++
                continue
            }
            
            # File has replacement chars when decoded as UTF-8 → it's GB2312 encoded
            Write-Host "  [CONVERT] $($file.Name) - GB2312 -> UTF-8"
            
            # Check if file is read-only
            $currentAttrs = (Get-Item $path).Attributes
            $wasReadOnly = $currentAttrs -band [System.IO.FileAttributes]::ReadOnly
            if ($wasReadOnly) {
                Set-ItemProperty -Path $path -Name IsReadOnly -Value $false
            }
            
            # Decode as GB2312, re-encode as UTF-8 (with BOM for better compatibility)
            $text = [System.Text.Encoding]::GetEncoding(936).GetString($bytes)
            [System.IO.File]::WriteAllText($path, $text, [System.Text.Encoding]::UTF8)
            
            # Restore read-only attribute if it was set
            if ($wasReadOnly) {
                Set-ItemProperty -Path $path -Name IsReadOnly -Value $true
            }
            
            $convertedCount++
        }
        catch {
            Write-Host "  [ERROR] $($file.Name): $_"
            $errorCount++
        }
    }
}

Write-Host "================================================"
Write-Host "Done! Converted: $convertedCount, Skipped: $skippedCount, Errors: $errorCount"
