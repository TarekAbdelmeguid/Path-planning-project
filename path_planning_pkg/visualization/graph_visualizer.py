import matplotlib.pyplot as plt
import networkx as nx

# Nodes: (x [m], y [m], node_id)
NODES = [
    (0.5, 0.6, 1), (1.5, 0.45, 2), (2.5, 0.45, 3), (3.55, 0.45, 4),
    (3.55, 1.5, 5), (3.4, 2.25, 6), (3.3, 3.5, 7), (3.4, 4.75, 8),
    (3.55, 5.5, 9), (3.55, 6.5, 10), (3.25, 7.5, 11), (2.5, 7.75, 12),
    (1.5, 7.75, 13), (0.5, 7.5, 14), (0.5, 6.5, 15), (0.5, 5.5, 16),
    (0.5, 4.75, 17), (0.5, 3.5, 18), (0.5, 2.25, 19), (0.5, 1.5, 20),
    (4.5, 0.45, 21), (5.5, 0.45, 22), (6.5, 0.45, 23), (7.25, 0.45, 24),
    (7.5, 1.5, 25), (7.25, 2.15, 26), (6.5, 2.15, 27), (5.5, 2.15, 28),
    (4.5, 2.15, 29), (2.5, 2.15, 30), (1.5, 2.25, 31), (2.5, 4.75, 32),
    (1.5, 4.75, 33)
]

# Edges: (from_node, to_node, cost)
EDGES = [
    (2, 1, 100), (3, 2, 100), (4, 3, 100), (4, 5, 100),
    (5, 6, 100), (6, 7, 100), (7, 8, 100), (8, 9, 100),
    (9, 10, 100), (10, 11, 100), (11, 12, 100), (12, 11, 100), (13, 12, 100),
    (14, 13, 100), (15, 14, 100), (16, 15, 100), (17, 16, 100),
    (18, 17, 100), (19, 18, 100), (20, 19, 100), (1, 20, 100),
    (21, 4, 100), (22, 21, 100), (23, 22, 100), (24, 23, 100),
    (25, 24, 100), (26, 25, 100), (27, 26, 100), (28, 27, 100),
    (29, 28, 100), (6, 29, 100), (30, 6, 100), (31, 30, 100),
    (19, 31, 100), (8, 32, 100), (32, 33, 100), (33, 17, 100), (17, 33, 100)
]

# Special node roles with colors
SPECIAL_NODES = {
    33: 'yellow',   # school
    11: 'red',      # hospital
    22: 'green',    # start
    2: 'blue',      # pickup 1
    28: 'pink'      # pickup 2
}

def plot_graph():
    print("List of nodes (ID: (x, y)):")
    for x, y, node_id in NODES:
        print(f"  Node {node_id}: ({x}, {y})")

    G = nx.DiGraph()
    pos = {}
    for x, y, node_id in NODES:
        G.add_node(node_id, pos=(x, y))
        pos[node_id] = (x, y)

    for start, end, cost in EDGES:
        G.add_edge(start, end, weight=cost)

    plt.figure(figsize=(14, 14))

    # Draw generic nodes
    other_nodes = [node_id for _, _, node_id in NODES if node_id not in SPECIAL_NODES]
    nx.draw_networkx_nodes(G, pos, nodelist=other_nodes, node_color='lightblue', node_size=500, edgecolors='black')

    # Draw edges
    nx.draw_networkx_edges(G, pos, width=2, edge_color='gray', arrows=True, arrowstyle='->', arrowsize=20)
    nx.draw_networkx_labels(G, pos, font_size=10, font_color='black')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=nx.get_edge_attributes(G, 'weight'))

    # Draw special nodes with shapes and labels
    shape_styles = {
        22: ('green', 'o', 'Start'),
        2:  ('blue', 'o', 'Pickup 1'),
        28: ('pink', 'o', 'Pickup 2'),
        11: ('red', 'o', 'Hospital'),
        33: ('yellow', 'o', 'School')
    }

    for node_id, (color, shape, label) in shape_styles.items():
        nx.draw_networkx_nodes(
            G, pos,
            nodelist=[node_id],
            node_color=color,
            node_shape=shape,
            node_size=800,
            edgecolors='black',
            label=label
    )
        # Add text label near the node with highlighted box
        x, y = pos[node_id]
        plt.text(
            x + -0.1, y + 0.3, label,
            fontsize=12, fontweight='bold', color='black',
            bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.3')
)


    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', label='Start (22)', markerfacecolor='green', markeredgecolor='black', markersize=15),
        Line2D([0], [0], marker='o', color='w', label='Pickup 1 (2)', markerfacecolor='blue', markeredgecolor='black', markersize=15),
        Line2D([0], [0], marker='o', color='w', label='Pickup 2 (28)', markerfacecolor='pink', markeredgecolor='black', markersize=15),
        Line2D([0], [0], marker='o', color='w', label='Hospital (11)', markerfacecolor='red', markeredgecolor='black', markersize=15),
        Line2D([0], [0], marker='o', color='w', label='School (33)', markerfacecolor='yellow', markeredgecolor='black', markersize=15),
        Line2D([0], [0], marker='o', color='w', label='Other Nodes', markerfacecolor='lightblue', markeredgecolor='black', markersize=15),
    ]
    plt.legend(handles=legend_elements, loc='upper right')

    plt.title("Model City Map - Directed Graph with Roles and Shapes")
    plt.axis('off')
    plt.tight_layout()
    plt.figtext(0.5, 0.01, 'By Traek Abdelmeguid', wrap=True, horizontalalignment='center', fontsize=12, fontstyle='italic')
    plt.savefig("modellstadt_colored_graph_with_labels_0.png", format="PNG")
    plt.show()

if __name__ == '__main__':
    plot_graph()

