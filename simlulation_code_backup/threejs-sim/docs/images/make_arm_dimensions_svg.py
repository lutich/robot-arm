"""Generates docs/images/arm-dimensions.svg. Run: python3 make_arm_dimensions_svg.py

Measured values are at the top. Servo bodies, brackets and the gripper outline follow the
photos and the app screenshot; only the dimensioned distances are measured.
"""
import math, os

S = 10.0  # px per cm
W, H = 1000, 1000
out = []
def add(s): out.append(s)

INK = "#1f2a44"; DIM = "#0a7d6b"; RED = "#c0392b"; LIGHT = "#dfe6ee"; GREY = "#8a94a6"
BLACK = "#24282f"; EDGE = "#0f1115"; SCREW = "#b9c0ca"; ORANGE = "#e8892b"
FONT = "font-family='Helvetica, Arial, sans-serif'"

# ---------------- measured values (cm) ----------------
BOARD_W, BOARD_D = 30.0, 38.0
AXIS_TO_EDGE = 1.5          # S1 axis to back edge of the board
H_S2 = 9.6                  # board top to S2 axis (whiteboard)
L_S2_S3 = 10.5              # team correction (Bild (3) showed 10,0)
L_S3_S4 = 12.6              # Bild (3)
L_S4_S5 = 6.7               # S4 horn to gripper flange, team correction (whiteboard 6,7)
G_OPEN_H, G_OPEN_GAP = 10.0, 4.5     # flange to finger tips / gap between pads, open
G_CLOSED_H, G_CLOSED_GAP = 11.5, 0.0 # closed, pads touch
G_BLOCK_H = 11.0                     # flange to block centre while holding the block
PAD_H = 1.2                          # height of the rubber pads on the finger tips
CUBE = BLOCK = 2.65

# ---------------- primitives ----------------
def line(x1,y1,x2,y2,stroke=INK,w=2,dash=None,cap="round"):
    d = f" stroke-dasharray='{dash}'" if dash else ""
    add(f"<line x1='{x1:.1f}' y1='{y1:.1f}' x2='{x2:.1f}' y2='{y2:.1f}' stroke='{stroke}' stroke-width='{w:.1f}' stroke-linecap='{cap}'{d}/>")
def rect(x,y,w,h,fill=LIGHT,stroke=INK,sw=2,dash=None,rx=0):
    d = f" stroke-dasharray='{dash}'" if dash else ""
    add(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='{rx}' fill='{fill}' stroke='{stroke}' stroke-width='{sw}'{d}/>")
def circle(cx,cy,r,fill="white",stroke=INK,sw=2,dash=None):
    d = f" stroke-dasharray='{dash}'" if dash else ""
    add(f"<circle cx='{cx:.1f}' cy='{cy:.1f}' r='{r:.1f}' fill='{fill}' stroke='{stroke}' stroke-width='{sw}'{d}/>")
def poly(pts,fill=BLACK,stroke=EDGE,sw=1.2):
    p = " ".join(f"{x:.1f},{y:.1f}" for x,y in pts)
    add(f"<polygon points='{p}' fill='{fill}' stroke='{stroke}' stroke-width='{sw}' stroke-linejoin='round'/>")
def text(x,y,s,size=13,fill=INK,anchor="middle",weight="normal",rot=0):
    r = f" transform='rotate({rot} {x:.1f} {y:.1f})'" if rot else ""
    add(f"<text x='{x:.1f}' y='{y:.1f}' font-size='{size}' fill='{fill}' text-anchor='{anchor}' font-weight='{weight}' {FONT}{r}>{s}</text>")
def screw(x,y,r=2.2): circle(x,y,r,SCREW,"#6b7380",0.8)
def horn(x,y,r=9):
    circle(x,y,r,"#d5dae1",EDGE,1.5); circle(x,y,r*0.45,"#9aa3ae",EDGE,1); circle(x,y,1.6,EDGE,EDGE,1)

def dim(x1,y1,x2,y2,label,off=18,color=DIM,flip=False):
    """Dimension line parallel to (x1,y1)-(x2,y2), offset perpendicular by `off` px."""
    dx,dy = x2-x1, y2-y1; L = math.hypot(dx,dy); ux,uy = dx/L, dy/L
    nx,ny = -uy, ux
    if flip: nx,ny = -nx,-ny
    ax,ay = x1+nx*off, y1+ny*off; bx,by = x2+nx*off, y2+ny*off
    line(x1,y1,ax+nx*4,ay+ny*4,color,1); line(x2,y2,bx+nx*4,by+ny*4,color,1)
    line(ax,ay,bx,by,color,1.5)
    for (px,py) in ((ax,ay),(bx,by)):
        line(px-nx*5,py-ny*5,px+nx*5,py+ny*5,color,1.5)
    ang = math.degrees(math.atan2(dy,dx))
    if ang>90 or ang<-90: ang += 180
    sg = 1 if off>=0 else -1
    if abs(abs(ang)-90) < 20:
        side = 1 if nx*sg >= 0 else -1
        text((ax+bx)/2+side*8,(ay+by)/2+4,label,12,color,"start" if side>0 else "end","bold")
    else:
        text((ax+bx)/2+nx*12*sg,(ay+by)/2+ny*12*sg+4,label,12,color,"middle","bold",ang)

# ---------------- gripper (claw) ----------------
# Gripper-local coords in cm: u to the right, v downward from the flange.
# Parallel linkage: each jaw hangs on a gear link and an outer link of length LINK,
# so the finger translates without turning. Geometry fitted to the photo values:
# open 6,8 / gap 4,7, closed 8,6 / gap 1,2.
LINK = 3.5
GEAR = (0.85, 1.4); OUTER = (2.2, 1.9)
TIP = (-1.515, 6.664)        # pad contact point at the finger tip, relative to the gear-link end
TH_OPEN, TH_CLOSED = 56.4, 11.0
def theta_for_gap(gap):
    return math.degrees(math.asin((gap/2 - GEAR[0] - TIP[0]) / LINK))
TH_BLOCK = theta_for_gap(BLOCK)
FINGER = [(0,0),(1.35,0.5),(1.25,3.4),(-0.9,6.75),(-1.25,6.70),(-1.25,5.2),(-0.5,3.1)]

def claw(cx, cy, sc, theta):
    """Draw the gripper hanging down from flange point (cx,cy). Returns tip points."""
    X = lambda u: cx + u*sc; Y = lambda v: cy + v*sc
    th = math.radians(theta)
    # flange and base plate
    rect(X(-1.0), Y(-0.6), 2.0*sc, 0.7*sc, "#cfd5dd", EDGE, 1, rx=2)
    poly([(X(-2.9),Y(0.1)),(X(2.9),Y(0.1)),(X(3.2),Y(1.2)),(X(2.5),Y(2.5)),(X(-2.5),Y(2.5)),(X(-3.2),Y(1.2))])
    tips = []
    for s in (-1, 1):
        G = (s*GEAR[0], GEAR[1]); O = (s*OUTER[0], OUTER[1])
        d = (s*math.sin(th)*LINK, math.cos(th)*LINK)
        K1 = (G[0]+d[0], G[1]+d[1]); K2 = (O[0]+d[0], O[1]+d[1])
        # gear
        circle(X(G[0]),Y(G[1]),0.8*sc,"#30353d","#59606b",max(1.5,0.25*sc),f"{0.18*sc:.1f},{0.14*sc:.1f}")
        # links
        line(X(O[0]),Y(O[1]),X(K2[0]),Y(K2[1]),EDGE,0.62*sc)
        line(X(O[0]),Y(O[1]),X(K2[0]),Y(K2[1]),BLACK,0.5*sc)
        line(X(G[0]),Y(G[1]),X(K1[0]),Y(K1[1]),EDGE,0.62*sc)
        line(X(G[0]),Y(G[1]),X(K1[0]),Y(K1[1]),BLACK,0.5*sc)
        # finger plate
        poly([(X(K1[0]+s*fu), Y(K1[1]+fv)) for fu,fv in FINGER])
        T = (K1[0]+s*TIP[0], K1[1]+TIP[1])
        pu = (T[0], T[0]+s*0.265)          # rubber pad on the inner face, PAD_H high
        rect(X(min(pu)), Y(T[1]-PAD_H), abs(pu[1]-pu[0])*sc, PAD_H*sc, "#e9ecef", "#8a94a6", 0.8, rx=1)
        for p in (G,O,K1,K2): screw(X(p[0]),Y(p[1]),max(1.6,0.2*sc))
        tips.append((X(T[0]),Y(T[1])))
    screw(X(0),Y(2.0),max(1.6,0.2*sc))
    return tips

# ======================================================================
add(f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {W} {H}' width='{W}' height='{H}'>")
add("<defs><linearGradient id='alu' x1='0' y1='0' x2='0' y2='1'>"
    "<stop offset='0' stop-color='#f4f6f8'/><stop offset='0.45' stop-color='#b9c0c9'/>"
    "<stop offset='1' stop-color='#7d8590'/></linearGradient></defs>")
add(f"<rect width='{W}' height='{H}' fill='white'/>")
text(30,34,"Robot arm dimensions",18,INK,"start","bold")
text(30,54,"All values in cm, measured servo horn to servo horn. Green = measured, red = whiteboard only. "
           "Servo bodies and brackets are drawn for shape only.",12,GREY,"start")

# ---------------- Panel 1: side view in the pose of the app screenshot ----------------
ox, oy = 500, 455   # S1 axis at board top surface; front of the board is to the LEFT
def P(y,z): return (ox - y*S, oy - z*S)
text(30,92,"1. Side view (pose as in the app screenshot)",15,INK,"start","bold")

def deg(a): return math.radians(a)
S2 = (0.0, H_S2); A1 = 70.0; A2 = 0.0
S3 = (S2[0]+L_S2_S3*math.cos(deg(A1)), S2[1]+L_S2_S3*math.sin(deg(A1)))
S4 = (S3[0]+L_S3_S4*math.cos(deg(A2)), S3[1]+L_S3_S4*math.sin(deg(A2)))
S5 = (S4[0], S4[1]-L_S4_S5)

# board with hole pattern
bx0,_ = P(BOARD_D-AXIS_TO_EDGE, 0)
rect(bx0, oy, BOARD_D*S, 1.0*S, "#3a3f48", EDGE, 1.2)
for i in range(1, 19):
    circle(bx0 + i*BOARD_D*S/19, oy+0.5*S, 1.6, "#6b7380", "#6b7380", 0.5)
text(bx0, oy+1.0*S+20, "board 38 deep (thickness not measured)", 11, GREY, "start")
dim(ox+AXIS_TO_EDGE*S, oy+1.0*S, ox, oy+1.0*S, "1,5", 22, RED, True)

# S1 housing, bracket, S2 servo
x0,y0 = P(1.5, 5.0); rect(x0, y0, 3.0*S, 5.0*S, BLACK, EDGE, 1.2, rx=2)
for (yy,zz) in ((-1.0,0.8),(1.0,0.8),(-1.0,4.2),(1.0,4.2)): screw(*P(yy,zz))
x0,y0 = P(1.2, 6.0); rect(x0, y0, 2.4*S, 1.0*S, BLACK, EDGE, 1.2)
x0,y0 = P(1.6, 11.4); rect(x0, y0, 3.2*S, 5.4*S, BLACK, EDGE, 1.2, rx=3)
line(*P(0,0.2), *P(0,H_S2), GREY, 1, "5,4")

# upper arm bracket S2 -> S3 with controller board and cables
(xa,ya),(xb,yb) = P(*S2), P(*S3)
line(xa,ya,xb,yb,EDGE,3.3*S); line(xa,ya,xb,yb,BLACK,3.0*S)
mx,my = (xa+xb)/2, (ya+yb)/2; ang = math.degrees(math.atan2(yb-ya, xb-xa))
add(f"<g transform='rotate({ang:.1f} {mx:.1f} {my:.1f})'>"
    f"<rect x='{mx-18:.1f}' y='{my-9:.1f}' width='36' height='18' rx='2' fill='#2f5fae' stroke='#e85d9c' stroke-width='2.5'/>"
    f"<rect x='{mx-12:.1f}' y='{my-4:.1f}' width='10' height='8' fill='#1f3f7a'/></g>")
add(f"<path d='M {ox+12:.1f} {oy-20:.1f} C {ox+30:.1f} {oy-120:.1f}, {xb+40:.1f} {yb+60:.1f}, {xb+14:.1f} {yb-6:.1f}' "
    f"fill='none' stroke='{ORANGE}' stroke-width='3'/>")
add(f"<path d='M {ox+16:.1f} {oy-20:.1f} C {ox+36:.1f} {oy-120:.1f}, {xb+46:.1f} {yb+62:.1f}, {xb+18:.1f} {yb-4:.1f}' "
    f"fill='none' stroke='#b0412b' stroke-width='2.5'/>")

# elbow servo, aluminium tube, wrist servos
x0,y0 = P(S3[0]+2.0, S3[1]+1.7); rect(x0, y0, 4.0*S, 3.6*S, BLACK, EDGE, 1.2, rx=4)
(t3x,t3y),(t4x,t4y) = P(*S3), P(*S4)
tl = math.hypot(t3x-t4x, t3y-t4y); ta = math.degrees(math.atan2(t3y-t4y, t3x-t4x))
add(f"<g transform='rotate({ta:.2f} {t4x:.1f} {t4y:.1f})'>")
rect(t4x+1.6*S, t4y-0.65*S, tl-3.6*S, 1.3*S, "url(#alu)", "#6b7380", 1)
for off in (2.0, tl/S-2.8):
    rect(t4x+off*S, t4y-1.0*S, 0.8*S, 2.0*S, BLACK, EDGE, 1)
add("</g>")
x0,y0 = P(S4[0]+1.6, S4[1]+1.6); rect(x0, y0, 3.2*S, (S4[1]+1.6-(S5[1]+1.2))*S, BLACK, EDGE, 1.2, rx=3)
line(*P(S4[0]+1.6, S4[1]-3.2), *P(S4[0]-1.6, S4[1]-3.2), "#4a515c", 1.2)
for (yy,zz) in ((S4[0]-1.0,S4[1]-3.8),(S4[0]+1.0,S4[1]-3.8),(S4[0]-1.0,S5[1]+1.7),(S4[0]+1.0,S5[1]+1.7)): screw(*P(yy,zz))
x0,y0 = P(S4[0]+0.9, S5[1]+1.2); rect(x0, y0, 1.8*S, 0.9*S, "url(#alu)", "#6b7380", 1)

# gripper hanging from S5 flange, open
tips = claw(*P(*S5), S, TH_OPEN)

# joint markers on the horns
for j in (S2,S3,S4): horn(*P(*j))
circle(*P(*S5), 4, "white", INK, 2)

# labels
x,y=P(*S2); text(x+30,y-14,"S2 shoulder",12,INK,"start","bold")
text(x+30,y+70,"S1 base rotation",12,INK,"start","bold"); text(x+30,y+84,"(vertical axis)",11,GREY,"start")
x,y=P(*S3); text(x+24,y-28,"S3 elbow",12,INK,"start","bold")
x,y=P(*S4); text(x-20,y-22,"S4 wrist flexion",12,INK,"end","bold")
x,y=P(*S5); text(x-56,y-2,"S5 wrist rotation",12,INK,"end","bold"); text(x-56,y+12,"(gripper flange)",11,GREY,"end")
x,y=P(*S5); text(x-5.6*S, tips[0][1]-10, "S6 gripper, open", 12, INK, "end", "bold")

# dimensions
dim(*P(1.6,0), *P(1.6,H_S2), "9,6", 14, RED, True)
dim(*P(*S2), *P(*S3), "10,5", 30, DIM, False)
dim(*P(*S3), *P(*S4), "12,6", 34, DIM, False)
dim(*P(*S4), *P(*S5), "6,7", 36, DIM, False)
x,y = P(*S5); dim(x+6.4*S, tips[0][1], x+6.4*S, y, "10,0", 4, DIM, False)

# ---------------- Panel 2: top view ----------------
tx0, ty0 = 640, 110
text(tx0,ty0-12,"2. Top view of the board",15,INK,"start","bold")
rect(tx0, ty0, BOARD_W*S, BOARD_D*S, LIGHT, INK)
cx,cy = tx0+BOARD_W/2*S, ty0+AXIS_TO_EDGE*S
circle(cx,cy,24,"#eef2f6",GREY,1.5)
circle(cx,cy,5,"white",INK,2); circle(cx,cy,2,INK,INK,1)
text(cx-30,cy+5,"S1 axis",11,INK,"end")
dim(cx,ty0,cx,cy,"1,5",34,RED,True)
text(cx, cy+40, "robot footprint not measured", 10, GREY)
text(cx, cy+54, "S1 axis centred on the 30 cm edge (assumed)", 10, GREY)
dim(tx0,ty0+BOARD_D*S,tx0+BOARD_W*S,ty0+BOARD_D*S,"30",-22,DIM,True)
dim(tx0+BOARD_W*S,ty0,tx0+BOARD_W*S,ty0+BOARD_D*S,"38",22,DIM,False)
line(cx,cy+68,cx,ty0+34*S,GREY,1.5,"5,5"); text(cx,ty0+36*S,"arm reach direction (assumed)",10,GREY)

# ---------------- Panel 3: gripper (2x scale) ----------------
GS = 20.0
gy0 = 598
text(30,gy0-44,"3. Gripper S6 (drawn 2x, 1 cm = 20 px)",15,INK,"start","bold")
def gripper_view(cx, theta, title):
    text(cx, gy0-22, title, 12, INK, "middle", "bold")
    return claw(cx, gy0, GS, theta)
c1 = 150
t = gripper_view(c1, TH_OPEN, "open")
dim(c1+5.6*GS, gy0+G_OPEN_H*GS, c1+5.6*GS, gy0, "10,0", 4)
dim(t[0][0], t[0][1]+12, t[1][0], t[1][1]+12, "4,5 gap", 14, DIM, False)
dim(t[0][0]+0.5*GS, t[0][1], t[0][0]+0.5*GS, t[0][1]-PAD_H*GS, "1,2 pad", 0, DIM, False)
c2 = 420
t = gripper_view(c2, TH_CLOSED, "closed")
dim(c2+4.2*GS, gy0+G_CLOSED_H*GS, c2+4.2*GS, gy0, "11,5", 4)
text(c2, t[0][1]+22, "gap 0, pads touch", 12, DIM, "middle", "bold")
c3 = 660
bx = c3 - BLOCK/2*GS; byc = gy0 + G_BLOCK_H*GS
rect(bx, byc-BLOCK/2*GS, BLOCK*GS, BLOCK*GS, "#ee7a3a", "#b4521f", 1.5, rx=3)
t = gripper_view(c3, TH_BLOCK, "holding block (11,0 = flange to block centre)")
circle(c3, byc, 3, "white", "#b4521f", 1)
dim(c3+4.6*GS, byc, c3+4.6*GS, gy0, "11,0", 4)
dim(bx, byc+BLOCK/2*GS+6, bx+BLOCK*GS, byc+BLOCK/2*GS+6, "2,65 block", 14, DIM, False)

# ---------------- Panel 4: cube (2x scale) ----------------
qx, qy = 870, 660
text(qx-30,gy0-44,"4. Test cube",15,INK,"start","bold")
e = CUBE*GS; d = e*0.45
add(f"<polygon points='{qx},{qy+d} {qx+d},{qy} {qx+e+d},{qy} {qx+e},{qy+d}' fill='#f39a63' stroke='#b4521f' stroke-width='1.5'/>")
add(f"<polygon points='{qx+e},{qy+d} {qx+e+d},{qy} {qx+e+d},{qy+e} {qx+e},{qy+d+e}' fill='#c9622c' stroke='#b4521f' stroke-width='1.5'/>")
rect(qx,qy+d,e,e,"#ee7a3a","#b4521f",1.5)
dim(qx,qy+d+e,qx+e,qy+d+e,"2,65",-16,DIM,True)
text(qx+e/2+10, qy+d+e+52, "all edges 2,65 (2x)", 11, GREY)

# ---------------- chain summary ----------------
sy = 900
text(30, sy, "Kinematic chain (distances between joint axes, cm)", 15, INK, "start", "bold")
chain = [("S1 base",None),("S2 shoulder","9,6 up"),("S3 elbow","10,5"),("S4 wrist flex","12,6"),("S5 flange","6,7"),("finger tips","10,0 open / 11,5 closed")]
x = 60
for i,(name,ln) in enumerate(chain):
    if ln:
        line(x-160, sy+40, x, sy+40, BLACK, 5)
        text(x-80, sy+30, ln, 12, RED if i==1 else DIM, "middle", "bold")
    circle(x, sy+40, 8, "white", INK, 2.5)
    text(x, sy+66, name, 11, INK)
    x += 170
add("</svg>")
here = os.path.dirname(os.path.abspath(__file__))
open(os.path.join(here, "arm-dimensions.svg"),"w").write("\n".join(out))
print("ok, block angle %.1f deg" % TH_BLOCK)
