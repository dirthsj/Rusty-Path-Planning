use petgraph::algo::astar;
use petgraph::graph::NodeIndex;
use petgraph::graph::UnGraph;
use petgraph::visit::Bfs;
use petgraph::{Graph, Undirected};

use std::fs::{read_to_string, write};
use std::ops::Add;

use serde::{Deserialize, Serialize};
use serde_json::json;

use clap::{App, Arg};

#[macro_use]
extern crate svgmacro;

fn main() -> std::io::Result<()> {
    let matches = App::new("My Test Program")
        .version("0.1.0")
        .author("Steven Dirth <steven@dirth.dev>")
        .about("Implements Simple Path Finding")
        .arg(
            Arg::with_name("input")
                .required(true)
                .short("i")
                .long("input")
                .takes_value(true)
                .help("The input configuration"),
        )
        .arg(
            Arg::with_name("json")
                .required_unless("svg")
                .short("j")
                .long("json")
                .takes_value(true)
                .help("The json file to write results to"),
        )
        .arg(
            Arg::with_name("svg")
                .required_unless("json")
                .short("s")
                .long("svg")
                .takes_value(true)
                .help("The svg file to write results to"),
        )
        .get_matches();

    let input_str = matches.value_of("input").unwrap();
    let str = read_to_string(input_str)?;

    let config = str_to_config(str);
    let g = make_graph(&config);
    let path = find_path(&g, config.start, config.goal);

    if let Some(json_path) = matches.value_of("json") {
        let json = graph_to_json(&g, &path);
        write(json_path, json)?;
    }

    if let Some(svg_path) = matches.value_of("svg") {
        let svg = graph_to_svg(&g, config.scale, &config, config.start, config.goal, &path);
        write(svg_path, svg)?;
    }

    Ok(())
}

#[derive(Copy, Clone, Serialize, Deserialize, Eq, PartialEq, Debug, PartialOrd, Default)]
struct Coordinate {
    x: i16,
    y: i16,
}

impl Coordinate {
    fn distance(self: Coordinate, other: Coordinate) -> i32 {
        (((self.y - other.y).pow(2) + (self.x - self.y).pow(2)) as f32).sqrt() as i32
    }
}

impl Add for Coordinate {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Config {
    start: Coordinate,
    goal: Coordinate,
    height: i16,
    width: i16,
    scale: i16,
}

fn make_graph(config: &Config) -> Graph<Coordinate, i32, Undirected, u32> {
    let field_height = config.height;
    let field_width = config.width;
    let node_count = field_height * field_width;
    let mut g = UnGraph::with_capacity(node_count as usize, (node_count * 2) as usize);
    let mut last_node_line = vec![None; field_width as usize];

    let mut y = 0;
    while y < field_height {
        let mut x = 0;
        let mut last_node = None;
        while x < field_width {
            let current_node = g.add_node(Coordinate { x, y });

            if last_node_line[x as usize].is_some() {
                g.add_edge(current_node, last_node_line[x as usize].unwrap(), 1);
            }
            if let Some(last_node_unwrapped) = last_node {
                g.add_edge(current_node, last_node_unwrapped, 1);
            }
            last_node = Some(current_node);
            last_node_line[x as usize] = Some(current_node);
            x += 1;
        }
        y += 1;
    }

    g
}

fn graph_to_svg(
    g: &Graph<Coordinate, i32, Undirected, u32>,
    scale: i16,
    config: &Config,
    start: Coordinate,
    goal: Coordinate,
    path: &Option<(i32, Vec<NodeIndex>)>,
) -> String {
    let field_width = config.width;
    let field_height = config.height;
    let offset = scale / 2;
    use std::fmt::Write;
    let mut out = String::new();
    let mut bfs = Bfs::new(&g, g.node_indices().next().unwrap());

    SVG!(&mut out,
        svg(width={field_width*scale + scale} height={field_height*scale + scale} xmlns="http://www.w3.org/2000/svg") [
            @ while let Some(nx) = bfs.next(&g) {
                let coord = g[nx];

                for neighbor in g.neighbors(nx) {
                    let ncoord = g[neighbor];
                    if let Some(path) = path {
                        if path.1.contains(&neighbor) && path.1.contains(&nx) {
                            SVG!(&mut out, line(x1={offset+scale*coord.x} y1={offset+scale*coord.y} x2={offset+scale*ncoord.x} y2={offset+scale*ncoord.y} style="stroke:rgb(0, 255, 0); stroke-width:2"));
                        } else {
                            SVG!(&mut out, line(x1={offset+scale*coord.x} y1={offset+scale*coord.y} x2={offset+scale*ncoord.x} y2={offset+scale*ncoord.y} style="stroke:rgb(0, 0, 0); stroke-width:1"));
                        }
                    } else {
                        SVG!(&mut out, line(x1={offset+scale*coord.x} y1={offset+scale*coord.y} x2={offset+scale*ncoord.x} y2={offset+scale*ncoord.y} style="stroke:rgb(0, 0, 0); stroke-width:1"));
                    }
                }
                if coord == start {
                    SVG!(&mut out, circle(cx={offset+scale*coord.x} cy={offset+scale*coord.y} r={scale/10} style="stroke: rgb(255, 0, 0); fill: rgb(255, 0, 0)"));
                } else if coord == goal {
                    SVG!(&mut out, circle(cx={offset+scale*coord.x} cy={offset+scale*coord.y} r={scale/10} style="stroke: rgb(0, 0, 255); fill: rgb(0, 0, 255)"));
                } else {
                    SVG!(&mut out, circle(cx={offset+scale*coord.x} cy={offset+scale*coord.y} r={scale/15}));
                }
            };
    ]);

    out
}

fn graph_to_json(
    g: &Graph<Coordinate, i32, Undirected, u32>,
    path: &Option<(i32, Vec<NodeIndex>)>,
) -> String {
    let nodes: Vec<Coordinate> = g.node_indices().map(|node_index| g[node_index]).collect();
    let mut bfs = Bfs::new(&g, g.node_indices().next().unwrap());
    let mut edges = Vec::new();
    while let Some(nx) = bfs.next(g) {
        let coord: Coordinate = g[nx];
        for neighbor in g.neighbors(nx) {
            let ncoord = g[neighbor];
            if !edges.contains(&(ncoord, coord)) {
                edges.push((coord, ncoord));
            }
        }
    }
    let out = if let Some(path) = path {
        json!({
            "nodes": nodes,
            "edges": edges,
            "path": path.1.iter().map(|x| g[*x]).collect::<Vec<Coordinate>>(),
        })
    } else {
        json!({
            "nodes": nodes,
            "edges": edges,
        })
    };
    serde_json::to_string(&out).unwrap()
}

fn find_path(
    g: &Graph<Coordinate, i32, Undirected, u32>,
    start: Coordinate,
    end: Coordinate,
) -> Option<(i32, Vec<NodeIndex>)> {
    if let Some(start_index) = g.node_indices().find(|x| g[*x] == start) {
        return astar(
            g,
            start_index,
            |x: NodeIndex| g[x] == end,
            |_x| 1,
            |x| end.distance(g[x]),
        );
    }
    None
}

fn str_to_config(str: String) -> Config {
    serde_json::from_str(str.as_str()).unwrap()
}
