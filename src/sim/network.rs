//! Road network: legs, lanes (links), movements, crosswalks, signal stages and the
//! conflict points between paths through the intersection, all derived from a
//! compact per-layout lane configuration.

use super::geom::{V2, Path, arc, connector, v2};
use serde::{Deserialize, Serialize};

pub type LinkId = usize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Compass {
    N,
    E,
    S,
    W,
}

impl Compass {
    pub const ALL: [Compass; 4] = [Compass::N, Compass::E, Compass::S, Compass::W];
    pub fn dir(self) -> V2 {
        match self {
            Compass::N => v2(0.0, -1.0),
            Compass::E => v2(1.0, 0.0),
            Compass::S => v2(0.0, 1.0),
            Compass::W => v2(-1.0, 0.0),
        }
    }
    pub fn letter(self) -> &'static str {
        match self {
            Compass::N => "N",
            Compass::E => "E",
            Compass::S => "S",
            Compass::W => "W",
        }
    }
    pub fn bound(self) -> &'static str {
        match self {
            Compass::N => "Southbound",
            Compass::E => "Westbound",
            Compass::S => "Northbound",
            Compass::W => "Eastbound",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Turn {
    L,
    T,
    R,
}

impl Turn {
    pub const ALL: [Turn; 3] = [Turn::L, Turn::T, Turn::R];
    pub fn letter(self) -> &'static str {
        match self {
            Turn::L => "L",
            Turn::T => "T",
            Turn::R => "R",
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Turn::L => "Left",
            Turn::T => "Through",
            Turn::R => "Right",
        }
    }
    pub fn index(self) -> usize {
        match self {
            Turn::L => 0,
            Turn::T => 1,
            Turn::R => 2,
        }
    }
    /// Outward direction of the exit leg for a vehicle entering from leg `from`.
    fn exit_dir(self, from: V2) -> V2 {
        match self {
            Turn::L => v2(-from.y, from.x),
            Turn::T => -from,
            Turn::R => v2(from.y, -from.x),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkKind {
    Inbound,
    Connector,
    Outbound,
    Ring,
    Entry,
    Exit,
}

impl LinkKind {
    /// Links inside the conflict area of the intersection.
    pub fn in_box(self) -> bool {
        matches!(self, LinkKind::Connector | LinkKind::Ring | LinkKind::Entry | LinkKind::Exit)
    }
}

#[derive(Clone, Debug)]
pub struct Link {
    pub id: LinkId,
    pub kind: LinkKind,
    pub path: Path,
    pub speed_limit: f64,
    pub curve_speed: f64,
    pub succ: Vec<LinkId>,
    pub pred: Vec<LinkId>,
    pub leg: usize,
    pub lane: usize,
    /// For inbound lanes: movements that may use this lane.
    pub lane_moves: Vec<usize>,
    /// For connectors: the movement they serve.
    pub movement: Option<usize>,
    pub conflicts: Vec<usize>,
    pub ped_conflicts: Vec<usize>,
    /// For a turn bay: distance along the lane where the bay opens (vehicles must
    /// change into it from the neighbouring lane). 0 for full-length lanes.
    pub bay_start: f64,
}

impl Link {
    pub fn length(&self) -> f64 {
        self.path.length
    }
}

#[derive(Clone, Debug)]
pub struct Conflict {
    pub id: usize,
    pub a: LinkId,
    pub sa: f64,
    pub b: LinkId,
    pub sb: f64,
    pub merge: bool,
    /// Zone along each path around the point where the bodies could touch:
    /// (distance before the point, distance after it).
    pub za: (f64, f64),
    pub zb: (f64, f64),
}

impl Conflict {
    /// (s on `link`, other link, s on other link)
    pub fn side(&self, link: LinkId) -> (f64, LinkId, f64) {
        if link == self.a { (self.sa, self.b, self.sb) } else { (self.sb, self.a, self.sa) }
    }
    /// Zones (mine, other's) as (before, after) for a vehicle on `link`.
    pub fn zones(&self, link: LinkId) -> ((f64, f64), (f64, f64)) {
        if link == self.a { (self.za, self.zb) } else { (self.zb, self.za) }
    }
}

/// Centre-line separation below which two vehicle bodies may touch (m).
const CLEAR: f64 = 2.7;

/// Interval of `pa` around `s0` that comes within `CLEAR` of `pb` near `sb0`.
fn zone_along(pa: &Path, s0: f64, pb: &Path, sb0: f64) -> (f64, f64) {
    let near = |s: f64| {
        let p = pa.point(s);
        let mut best = f64::INFINITY;
        let mut sb = (sb0 - 18.0).max(0.0);
        while sb <= (sb0 + 18.0).min(pb.length) {
            best = best.min(p.dist(pb.point(sb)));
            sb += 0.5;
        }
        best < CLEAR
    };
    let mut lo = 0.0;
    while lo < 20.0 && s0 - lo - 0.5 >= 0.0 && near(s0 - lo - 0.5) {
        lo += 0.5;
    }
    let mut hi = 0.0;
    while hi < 20.0 && s0 + hi + 0.5 <= pa.length && near(s0 + hi + 0.5) {
        hi += 0.5;
    }
    (lo + 0.6, hi + 0.6)
}

#[derive(Clone, Debug)]
pub struct PedConflict {
    pub link: LinkId,
    pub s: f64,
    pub cw: usize,
    pub u: f64,
}

#[derive(Clone, Debug)]
pub struct Crosswalk {
    pub id: usize,
    pub leg: usize,
    pub a: V2,
    pub b: V2,
    pub length: f64,
}

#[derive(Clone, Debug)]
pub struct Leg {
    pub compass: Compass,
    pub d: V2,
    pub lane_moves: Vec<Vec<Turn>>,
    pub n_out: usize,
    pub lane_w: f64,
    pub med_out: f64,
    pub in_width: f64,
    pub out_width: f64,
    pub stop_dist: f64,
    pub speed: f64,
    pub in_lanes: Vec<LinkId>,
    pub out_lanes: Vec<LinkId>,
    pub crosswalk: Option<usize>,
    /// Minor approach in two-way-stop control.
    pub minor: bool,
}

impl Leg {
    /// Lateral unit vector toward the inbound side.
    pub fn in_side(&self) -> V2 {
        v2(self.d.y, -self.d.x)
    }
    pub fn out_side(&self) -> V2 {
        -self.in_side()
    }
    pub fn turns(&self) -> Vec<Turn> {
        let mut t: Vec<Turn> = Vec::new();
        for lm in &self.lane_moves {
            for &x in lm {
                if !t.contains(&x) {
                    t.push(x);
                }
            }
        }
        t
    }
}

#[derive(Clone, Debug)]
pub struct Movement {
    pub id: usize,
    pub name: String,
    pub leg: usize,
    pub turn: Turn,
    pub to_leg: usize,
    /// (inbound lane, rest of route after the inbound lane)
    pub routes: Vec<(LinkId, Vec<LinkId>)>,
    pub crosswalks: Vec<usize>,
}

/// Signal indication class of a movement within a stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MStat {
    /// Protected (green ball / green arrow).
    G,
    /// Protected from vehicles but yields to pedestrians (right turns).
    Rt,
    /// Permissive: yields to opposing vehicles and pedestrians.
    P,
}

#[derive(Clone, Debug)]
pub struct Stage {
    pub name: String,
    pub stat: Vec<Option<MStat>>,
    pub walk: Vec<usize>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LayoutKind {
    Arterial4,
    MajorMinor4,
    Town4,
    Tee,
    Roundabout,
}

impl LayoutKind {
    pub const ALL: [LayoutKind; 5] =
        [LayoutKind::Arterial4, LayoutKind::MajorMinor4, LayoutKind::Town4, LayoutKind::Tee, LayoutKind::Roundabout];
    pub fn name(self) -> &'static str {
        match self {
            LayoutKind::Arterial4 => "4-Way Arterial (left-turn bays)",
            LayoutKind::MajorMinor4 => "4-Way Major / Minor",
            LayoutKind::Town4 => "4-Way Single-Lane (town)",
            LayoutKind::Tee => "T-Junction",
            LayoutKind::Roundabout => "Roundabout (single-lane)",
        }
    }
    pub fn key(self) -> &'static str {
        match self {
            LayoutKind::Arterial4 => "arterial4",
            LayoutKind::MajorMinor4 => "majorminor4",
            LayoutKind::Town4 => "town4",
            LayoutKind::Tee => "tee",
            LayoutKind::Roundabout => "roundabout",
        }
    }
    pub fn signalized(self) -> bool {
        self != LayoutKind::Roundabout
    }
}

#[derive(Clone, Debug)]
pub struct Network {
    pub kind: LayoutKind,
    pub legs: Vec<Leg>,
    pub links: Vec<Link>,
    pub movements: Vec<Movement>,
    pub conflicts: Vec<Conflict>,
    pub ped_conflicts: Vec<PedConflict>,
    pub crosswalks: Vec<Crosswalk>,
    pub stages: Vec<Stage>,
    pub fixed_sequence: Vec<usize>,
    pub yellow: f64,
    pub all_red: f64,
    pub in_len: f64,
    pub out_len: f64,
    /// Roundabout ring centre radius (0 if none).
    pub ring_r: f64,
    pub ring_w: f64,
}

pub const IN_LEN: f64 = 260.0;
/// Length of a left-turn bay (storage + taper) before the stop line.
pub const BAY_LEN: f64 = 75.0;
pub const OUT_LEN: f64 = 160.0;
const A_LAT: f64 = 2.6;

impl Network {
    pub fn leg_index(&self, c: Compass) -> Option<usize> {
        self.legs.iter().position(|l| l.compass == c)
    }
    pub fn movement_by_name(&self, name: &str) -> Option<usize> {
        self.movements.iter().position(|m| m.name == name)
    }
    pub fn movement_for(&self, leg: usize, turn: Turn) -> Option<usize> {
        self.movements.iter().position(|m| m.leg == leg && m.turn == turn)
    }
    pub fn stop_link(&self, link: LinkId) -> bool {
        self.links[link].kind == LinkKind::Inbound
    }

    pub fn build(kind: LayoutKind) -> Network {
        match kind {
            LayoutKind::Arterial4 => {
                let lanes = || vec![vec![Turn::L], vec![Turn::T], vec![Turn::T, Turn::R]];
                let legs = Compass::ALL.iter().map(|&c| (c, lanes(), 2, false)).collect();
                let mut n = build_signal(kind, legs, 3.5, 13.9, 3.5, 2.0);
                n.add_stages(&[
                    ("NS Through", &["N_T", "S_T"], &["N_R", "S_R"], &["N_L", "S_L"]),
                    ("NS Protected Left", &["N_L", "S_L"], &["N_R", "S_R"], &[]),
                    ("EW Through", &["E_T", "W_T"], &["E_R", "W_R"], &["E_L", "W_L"]),
                    ("EW Protected Left", &["E_L", "W_L"], &["E_R", "W_R"], &[]),
                    ("North Split", &["N_L", "N_T"], &["N_R"], &[]),
                    ("South Split", &["S_L", "S_T"], &["S_R"], &[]),
                    ("East Split", &["E_L", "E_T"], &["E_R"], &[]),
                    ("West Split", &["W_L", "W_T"], &["W_R"], &[]),
                ]);
                n.fixed_sequence = vec![1, 0, 3, 2];
                n
            }
            LayoutKind::MajorMinor4 => {
                let major = || vec![vec![Turn::L], vec![Turn::T], vec![Turn::T, Turn::R]];
                let minor = || vec![vec![Turn::L, Turn::T, Turn::R]];
                let legs = vec![
                    (Compass::N, minor(), 1, true),
                    (Compass::E, major(), 2, false),
                    (Compass::S, minor(), 1, true),
                    (Compass::W, major(), 2, false),
                ];
                let mut n = build_signal(kind, legs, 3.5, 13.9, 3.5, 2.0);
                n.add_stages(&[
                    ("EW Through", &["E_T", "W_T"], &["E_R", "W_R"], &["E_L", "W_L"]),
                    ("EW Protected Left", &["E_L", "W_L"], &["E_R", "W_R"], &[]),
                    ("East Split", &["E_L", "E_T"], &["E_R"], &[]),
                    ("West Split", &["W_L", "W_T"], &["W_R"], &[]),
                    ("NS Side Street", &["N_T", "S_T"], &["N_R", "S_R"], &["N_L", "S_L"]),
                    ("North Split", &["N_L", "N_T"], &["N_R"], &[]),
                    ("South Split", &["S_L", "S_T"], &["S_R"], &[]),
                ]);
                n.fixed_sequence = vec![1, 0, 4];
                n
            }
            LayoutKind::Town4 => {
                let one = || vec![vec![Turn::L, Turn::T, Turn::R]];
                let legs = Compass::ALL.iter().map(|&c| (c, one(), 1, false)).collect();
                let mut n = build_signal(kind, legs, 3.4, 11.1, 3.0, 1.5);
                n.add_stages(&[
                    ("NS (permissive)", &["N_T", "S_T"], &["N_R", "S_R"], &["N_L", "S_L"]),
                    ("EW (permissive)", &["E_T", "W_T"], &["E_R", "W_R"], &["E_L", "W_L"]),
                    ("North Split", &["N_L", "N_T"], &["N_R"], &[]),
                    ("South Split", &["S_L", "S_T"], &["S_R"], &[]),
                    ("East Split", &["E_L", "E_T"], &["E_R"], &[]),
                    ("West Split", &["W_L", "W_T"], &["W_R"], &[]),
                ]);
                n.fixed_sequence = vec![0, 1];
                n
            }
            LayoutKind::Tee => {
                let legs = vec![
                    (Compass::E, vec![vec![Turn::L], vec![Turn::T], vec![Turn::T]], 2, false),
                    (Compass::S, vec![vec![Turn::L], vec![Turn::R]], 1, true),
                    (Compass::W, vec![vec![Turn::T], vec![Turn::T, Turn::R]], 2, false),
                ];
                let mut n = build_signal(kind, legs, 3.5, 13.9, 3.5, 2.0);
                n.add_stages(&[
                    ("Main Street", &["E_T", "W_T"], &["W_R"], &["E_L"]),
                    ("East + Protected Left", &["E_T", "E_L"], &[], &[]),
                    ("Side Street", &[], &["S_L", "S_R"], &[]),
                    ("Left In + Right Out", &["E_L"], &["S_R"], &[]),
                ]);
                n.fixed_sequence = vec![0, 1, 2];
                n
            }
            LayoutKind::Roundabout => build_roundabout(),
        }
    }

    fn add_stages(&mut self, defs: &[(&str, &[&str], &[&str], &[&str])]) {
        for (name, g, rt, p) in defs {
            let mut stat = vec![None; self.movements.len()];
            for (list, st) in [(*g, MStat::G), (*rt, MStat::Rt), (*p, MStat::P)] {
                for m in list.iter() {
                    if let Some(i) = self.movement_by_name(m) {
                        stat[i] = Some(st);
                    }
                }
            }
            let walk = self.walk_for(&stat);
            self.stages.push(Stage { name: name.to_string(), stat, walk });
        }
    }

    /// Crosswalks that may walk during a stage: all except those a protected (G)
    /// movement crosses or that any served movement enters the intersection across.
    pub fn walk_for(&self, stat: &[Option<MStat>]) -> Vec<usize> {
        (0..self.crosswalks.len())
            .filter(|&cw| {
                let leg = self.crosswalks[cw].leg;
                !self.movements.iter().any(|m| {
                    stat[m.id].is_some() && m.crosswalks.contains(&cw) && (stat[m.id] == Some(MStat::G) || m.leg == leg)
                })
            })
            .collect()
    }

    /// Left turns that can run protected-only: a dedicated left lane, a stage with a
    /// green arrow and a stage where the left is permissive (flashing yellow arrow).
    /// `stages` is the original stage plan (before any left is made protected-only).
    pub fn fya_eligible(&self, stages: &[Stage], m: usize) -> bool {
        let mv = &self.movements[m];
        mv.turn == Turn::L
            && self.legs[mv.leg].in_lanes.iter().any(|&l| self.links[l].lane_moves == [m])
            && stages.iter().any(|s| s.stat[m] == Some(MStat::G))
            && stages.iter().any(|s| s.stat[m] == Some(MStat::P))
    }
}

fn new_link(links: &mut Vec<Link>, kind: LinkKind, pts: &[V2], speed: f64, leg: usize, lane: usize) -> LinkId {
    let path = Path::new(pts);
    let curve_speed = if path.min_radius.is_finite() {
        (A_LAT * path.min_radius).sqrt().clamp(3.5, speed)
    } else {
        speed
    };
    let id = links.len();
    links.push(Link {
        id,
        kind,
        path,
        speed_limit: speed,
        curve_speed,
        succ: vec![],
        pred: vec![],
        leg,
        lane,
        lane_moves: vec![],
        movement: None,
        conflicts: vec![],
        ped_conflicts: vec![],
        bay_start: 0.0,
    });
    id
}

fn connect(links: &mut [Link], a: LinkId, b: LinkId) {
    if !links[a].succ.contains(&b) {
        links[a].succ.push(b);
        links[b].pred.push(a);
    }
}

type LegDef = (Compass, Vec<Vec<Turn>>, usize, bool);

fn build_signal(kind: LayoutKind, defs: Vec<LegDef>, lane_w: f64, speed: f64, yellow: f64, all_red: f64) -> Network {
    let mut legs: Vec<Leg> = defs
        .into_iter()
        .map(|(c, lm, n_out, minor)| Leg {
            compass: c,
            d: c.dir(),
            in_width: lm.len() as f64 * lane_w,
            lane_moves: lm,
            n_out,
            lane_w,
            med_out: 0.0,
            out_width: 0.0,
            stop_dist: 0.0,
            speed,
            in_lanes: vec![],
            out_lanes: vec![],
            crosswalk: None,
            minor,
        })
        .collect();
    let find = |legs: &Vec<Leg>, d: V2| legs.iter().position(|l| l.d.dist(d) < 1e-6);
    // Align outbound lanes with the opposite approach's through lanes.
    for i in 0..legs.len() {
        let opp = find(&legs, -legs[i].d);
        let med = opp
            .map(|o| legs[o].lane_moves.iter().position(|lm| lm.contains(&Turn::T)).unwrap_or(0) as f64 * lane_w)
            .unwrap_or(0.0);
        legs[i].med_out = med;
        legs[i].out_width = med + legs[i].n_out as f64 * lane_w;
    }
    for i in 0..legs.len() {
        let d = legs[i].d;
        let mut hw: f64 = 0.0;
        for l in &legs {
            if l.d.dot(d).abs() < 0.5 {
                hw = hw.max(l.in_width.max(l.out_width));
            }
        }
        legs[i].stop_dist = hw + 6.0;
    }

    let mut links: Vec<Link> = Vec::new();
    for (li, leg) in legs.iter_mut().enumerate() {
        let (d, ins, outs) = (leg.d, leg.in_side(), leg.out_side());
        for lane in 0..leg.lane_moves.len() {
            let lat = ins * ((lane as f64 + 0.5) * lane_w);
            let p0 = d * IN_LEN + lat;
            let p1 = d * leg.stop_dist + lat;
            let id = new_link(&mut links, LinkKind::Inbound, &[p0, p1], speed, li, lane);
            leg.in_lanes.push(id);
        }
        // A left-only lane next to a through lane is a bay carved out of the median.
        if leg.lane_moves.len() >= 2 && leg.lane_moves[0] == [Turn::L] && leg.lane_moves[1].contains(&Turn::T) {
            let l = &mut links[leg.in_lanes[0]];
            l.bay_start = (l.path.length - BAY_LEN).max(0.0);
        }
        for lane in 0..leg.n_out {
            let lat = outs * (leg.med_out + (lane as f64 + 0.5) * lane_w);
            let p0 = d * leg.stop_dist + lat;
            let p1 = d * OUT_LEN + lat;
            let id = new_link(&mut links, LinkKind::Outbound, &[p0, p1], speed, li, lane);
            leg.out_lanes.push(id);
        }
    }

    let mut movements: Vec<Movement> = Vec::new();
    for li in 0..legs.len() {
        let d = legs[li].d;
        let mut through_k = 0;
        for lane in 0..legs[li].lane_moves.len() {
            let turns = legs[li].lane_moves[lane].clone();
            let in_id = legs[li].in_lanes[lane];
            for &turn in &turns {
                let Some(to) = find(&legs, turn.exit_dir(d)) else { continue };
                let n_out = legs[to].n_out;
                let out_lane = match turn {
                    Turn::L => 0,
                    Turn::R => n_out - 1,
                    Turn::T => through_k.min(n_out - 1),
                };
                let out_id = legs[to].out_lanes[out_lane];
                let p0 = links[in_id].path.end();
                let p3 = links[out_id].path.start();
                let pts = connector(p0, -d, p3, legs[to].d);
                let c = new_link(&mut links, LinkKind::Connector, &pts, speed, li, lane);
                connect(&mut links, in_id, c);
                connect(&mut links, c, out_id);
                let mid = match movements.iter().position(|m| m.leg == li && m.turn == turn) {
                    Some(m) => m,
                    None => {
                        movements.push(Movement {
                            id: movements.len(),
                            name: format!("{}_{}", legs[li].compass.letter(), turn.letter()),
                            leg: li,
                            turn,
                            to_leg: to,
                            routes: vec![],
                            crosswalks: vec![],
                        });
                        movements.len() - 1
                    }
                };
                movements[mid].routes.push((in_id, vec![c, out_id]));
                links[c].movement = Some(mid);
                links[in_id].lane_moves.push(mid);
            }
            if turns.contains(&Turn::T) {
                through_k += 1;
            }
        }
    }

    // Crosswalks across each leg, just inside the stop line.
    let mut crosswalks = Vec::new();
    for (li, leg) in legs.iter_mut().enumerate() {
        let r = leg.stop_dist - 3.0;
        let a = leg.d * r + leg.out_side() * (leg.out_width + 0.4);
        let b = leg.d * r + leg.in_side() * (leg.in_width + 0.4);
        leg.crosswalk = Some(crosswalks.len());
        crosswalks.push(Crosswalk { id: crosswalks.len(), leg: li, a, b, length: a.dist(b) });
    }

    let mut net = Network {
        kind,
        legs,
        links,
        movements,
        conflicts: vec![],
        ped_conflicts: vec![],
        crosswalks,
        stages: vec![],
        fixed_sequence: vec![],
        yellow,
        all_red,
        in_len: IN_LEN,
        out_len: OUT_LEN,
        ring_r: 0.0,
        ring_w: 0.0,
    };
    net.compute_conflicts();
    net
}

fn build_roundabout() -> Network {
    let lane_w = 3.6;
    let speed = 13.9;
    let ring_r = 14.0;
    let ring_w = 5.5;
    let lat = 3.4;
    let yield_r = ring_r + ring_w / 2.0 + 5.0;
    let order = [Compass::N, Compass::W, Compass::S, Compass::E]; // counter-clockwise on screen
    let mut legs: Vec<Leg> = order
        .iter()
        .map(|&c| Leg {
            compass: c,
            d: c.dir(),
            lane_moves: vec![vec![Turn::L, Turn::T, Turn::R]],
            n_out: 1,
            lane_w,
            med_out: lat - lane_w / 2.0,
            in_width: lat + lane_w / 2.0,
            out_width: lat + lane_w / 2.0,
            stop_dist: yield_r,
            speed,
            in_lanes: vec![],
            out_lanes: vec![],
            crosswalk: None,
            minor: false,
        })
        .collect();
    let mut links: Vec<Link> = Vec::new();
    let delta: f64 = 0.62;
    let c0 = v2(0.0, 0.0);
    let tan = |a: f64| v2(a.sin(), -a.cos());
    let n = legs.len();
    let mut entry = vec![0; n];
    let mut exit = vec![0; n];
    let mut seg_a = vec![0; n];
    let mut seg_b = vec![0; n];
    for (i, leg) in legs.iter_mut().enumerate() {
        let d = leg.d;
        let th = d.angle();
        let pin0 = d * IN_LEN + leg.in_side() * lat;
        let pin1 = d * yield_r + leg.in_side() * lat;
        let inl = new_link(&mut links, LinkKind::Inbound, &[pin0, pin1], speed, i, 0);
        leg.in_lanes.push(inl);
        let pout0 = d * yield_r + leg.out_side() * lat;
        let pout1 = d * OUT_LEN + leg.out_side() * lat;
        let outl = new_link(&mut links, LinkKind::Outbound, &[pout0, pout1], speed, i, 0);
        leg.out_lanes.push(outl);
        let ae = th - delta;
        let ax = th + delta;
        let pe = V2::from_angle(ae) * ring_r;
        let px = V2::from_angle(ax) * ring_r;
        entry[i] = new_link(&mut links, LinkKind::Entry, &connector(pin1, -d, pe, tan(ae)), speed, i, 0);
        exit[i] = new_link(&mut links, LinkKind::Exit, &connector(px, tan(ax), pout0, d), speed, i, 0);
        seg_a[i] = new_link(&mut links, LinkKind::Ring, &arc(c0, ring_r, ax, ae), speed, i, 0);
        connect(&mut links, inl, entry[i]);
        connect(&mut links, exit[i], outl);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let th_i = legs[i].d.angle();
        let mut th_j = legs[j].d.angle();
        while th_j > th_i - delta {
            th_j -= std::f64::consts::TAU;
        }
        seg_b[i] = new_link(&mut links, LinkKind::Ring, &arc(c0, ring_r, th_i - delta, th_j + delta), speed, i, 0);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        connect(&mut links, entry[i], seg_b[i]);
        connect(&mut links, seg_a[i], seg_b[i]);
        connect(&mut links, seg_b[i], exit[j]);
        connect(&mut links, seg_b[i], seg_a[j]);
    }
    let mut movements = Vec::new();
    for i in 0..n {
        for (k, turn) in [(1, Turn::R), (2, Turn::T), (3, Turn::L)] {
            let to = (i + k) % n;
            let mut route = vec![entry[i], seg_b[i]];
            let mut c = (i + 1) % n;
            while c != to {
                route.push(seg_a[c]);
                route.push(seg_b[c]);
                c = (c + 1) % n;
            }
            route.push(exit[to]);
            route.push(legs[to].out_lanes[0]);
            let id = movements.len();
            let inl = legs[i].in_lanes[0];
            links[inl].lane_moves.push(id);
            movements.push(Movement {
                id,
                name: format!("{}_{}", legs[i].compass.letter(), turn.letter()),
                leg: i,
                turn,
                to_leg: to,
                routes: vec![(inl, route)],
                crosswalks: vec![],
            });
        }
    }
    let mut net = Network {
        kind: LayoutKind::Roundabout,
        legs,
        links,
        movements,
        conflicts: vec![],
        ped_conflicts: vec![],
        crosswalks: vec![],
        stages: vec![],
        fixed_sequence: vec![],
        yellow: 0.0,
        all_red: 0.0,
        in_len: IN_LEN,
        out_len: OUT_LEN,
        ring_r,
        ring_w,
    };
    net.compute_conflicts();
    net
}

impl Network {
    fn compute_conflicts(&mut self) {
        let boxed: Vec<LinkId> = self.links.iter().filter(|l| l.kind.in_box()).map(|l| l.id).collect();
        let mut conflicts: Vec<Conflict> = Vec::new();
        for (ii, &a) in boxed.iter().enumerate() {
            for &b in &boxed[ii + 1..] {
                let la = &self.links[a];
                let lb = &self.links[b];
                let adjacent = la.succ.contains(&b) || lb.succ.contains(&a);
                let diverge = la.pred.iter().any(|p| lb.pred.contains(p));
                let merge = la.succ.iter().any(|s| lb.succ.contains(s));
                if adjacent || diverge {
                    continue;
                }
                let mut pts: Vec<(f64, f64, bool)> = Vec::new();
                if merge {
                    pts.push((la.length(), lb.length(), true));
                } else {
                    let mut xs = la.path.intersections(&lb.path);
                    if xs.is_empty() {
                        let (sa, sb, dist) = la.path.closest(&lb.path);
                        if dist < CLEAR {
                            xs.push((sa, sb));
                        }
                    }
                    pts.extend(xs.into_iter().map(|(x, y)| (x, y, false)));
                }
                for (sa, sb, merge) in pts {
                    let mut za = zone_along(&la.path, sa, &lb.path, sb);
                    let mut zb = zone_along(&lb.path, sb, &la.path, sa);
                    if merge {
                        // After merging the vehicles share a lane and simply follow.
                        za.1 = 1.0;
                        zb.1 = 1.0;
                    }
                    conflicts.push(Conflict { id: conflicts.len(), a, sa, b, sb, merge, za, zb });
                }
            }
        }
        for c in &conflicts {
            self.links[c.a].conflicts.push(c.id);
            self.links[c.b].conflicts.push(c.id);
        }
        for l in self.links.iter_mut() {
            let id = l.id;
            l.conflicts.sort_by(|&x, &y| {
                conflicts[x].side(id).0.partial_cmp(&conflicts[y].side(id).0).unwrap()
            });
        }
        self.conflicts = conflicts;

        let mut pcs = Vec::new();
        for &lid in &boxed {
            for cw in &self.crosswalks {
                let seg = Path::new(&[cw.a, cw.b]);
                for (s, u) in self.links[lid].path.intersections(&seg) {
                    pcs.push(PedConflict { link: lid, s, cw: cw.id, u });
                }
            }
        }
        for (i, pc) in pcs.iter().enumerate() {
            self.links[pc.link].ped_conflicts.push(i);
            if let Some(m) = self.links[pc.link].movement {
                if !self.movements[m].crosswalks.contains(&pc.cw) {
                    self.movements[m].crosswalks.push(pc.cw);
                }
            }
        }
        self.ped_conflicts = pcs;
    }
}
