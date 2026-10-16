from pathlib import Path
from PIL import Image, ImageDraw
p=Path(__file__).resolve().parent.parent;src=p/'source'
for f in [src/'AndroidManifest.xml',src/'assets/index.html',p/'preview/app.html']:
 s=f.read_text(encoding='utf-8').replace('青羽飞控','LFish · 青鳍');f.write_text(s,encoding='utf-8')
# Deterministic project-native icon, with fish body and four propeller rings.
im=Image.new('RGB',(1024,1024)); pix=im.load()
for y in range(1024):
 for x in range(1024):
  t=(x+y)/2048;pix[x,y]=(int(12+12*t),int(70+55*t),int(76+34*t))
d=ImageDraw.Draw(im)
for a,b in [((310,310),(714,714)),((714,310),(310,714))]: d.line([a,b],fill='#85c9b9',width=34)
for x,y in [(310,310),(714,310),(310,714),(714,714)]:
 d.ellipse((x-132,y-132,x+132,y+132),outline='#dff7f0',width=22)
 d.arc((x-105,y-105,x+105,y+105),190,290,fill='#76baac',width=10)
d.polygon([(504,290),(604,423),(595,578),(516,668),(436,578),(426,423)],fill='#f2fffb')
d.polygon([(516,658),(438,744),(516,721),(594,744)],fill='#b8e8da')
d.polygon([(440,455),(373,518),(436,532)],fill='#c3ecdf')
d.polygon([(596,455),(660,518),(595,532)],fill='#c3ecdf')
d.ellipse((488,378,520,410),fill='#176d67')
assets=p/'assets';assets.mkdir(exist_ok=True);im.save(assets/'lfish-icon.png')
for dest in [src/'assets/app-icon.png',p/'preview/app-icon.png']:
 im.resize((512,512),Image.Resampling.LANCZOS).save(dest)
im.resize((192,192),Image.Resampling.LANCZOS).save(src/'res/drawable/app_icon.png')
(src/'res/drawable/app_icon.xml').unlink(missing_ok=True)
print('Brand assets saved')
