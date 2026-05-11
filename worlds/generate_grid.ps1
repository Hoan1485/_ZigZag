Add-Type -AssemblyName System.Drawing
$bmp = New-Object System.Drawing.Bitmap(1500, 1500)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.Clear([System.Drawing.Color]::Transparent)
$pen = New-Object System.Drawing.Pen([System.Drawing.Color]::Black, 8)
for ($i=0; $i -le 15; $i++) {
    $pos = $i * 100
    $g.DrawLine($pen, 0, $pos, 1500, $pos)
    $g.DrawLine($pen, $pos, 0, $pos, 1500)
}
$bmp.Save("grid.png", [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose()
$bmp.Dispose()
