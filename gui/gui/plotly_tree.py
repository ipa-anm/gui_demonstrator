import igraph
from igraph import Graph, EdgeSeq
import plotly.graph_objects as go
import random

number_child_nodes=8
size = [90]+([70]*(number_child_nodes-1))
color_name_choices = ["Pick<br>Blue", "Pick<br>Red", "Pick<br>Green", "Pick<br>Yellow", "Pick<br>Red"]
color_choices = ["blue", "yellow", "red", "green"]
color = ["black"]+random.choices(color_choices, k=number_child_nodes-1)
color_names = [c +  "<br>pick" for c in color]

def make_annotations(pos, text, M, font_size=20, font_color='rgb(250,250,250)'):
    L=len(pos)
    print(pos)
    print(pos[0][0])
    if len(text)!=L:
        raise ValueError('The lists pos and text must have the same len')
    annotations = []
    annotations.append(dict(
                text="Static<br>Scenario", # or replace labels with a different list for the text within the circle
                x=pos[0][0], y=2*M-pos[0][1],
                xref='x1', yref='y1',
                font=dict(color="white", size=20),
                showarrow=False))
    for k in range(L-1):
        annotations.append(
            dict(
                text=color_names[k+1], # or replace labels with a different list for the text within the circle
                x=pos[k+1][0], y=2*M-pos[k+1][1],
                xref='x1', yref='y1',
                font=dict(color="black", size=font_size),
                showarrow=False)
        )
    return annotations

def create_tree():
    nr_vertices = number_child_nodes
    v_label = list(map(str, range(nr_vertices)))
    print(v_label)
    G = Graph.Tree(nr_vertices, (nr_vertices-1)) # 2 stands for children number
    lay = G.layout('rt')

    position = {k: lay[k] for k in range(nr_vertices)}
    Y = [lay[k][1] for k in range(nr_vertices)]
    M = max(Y)

    es = EdgeSeq(G) # sequence of edges
    E = [e.tuple for e in G.es] # list of edges

    L = len(position)
    Xn = [position[k][0] for k in range(L)]
    Yn = [2*M-position[k][1] for k in range(L)]
    Xe = []
    Ye = []
    for edge in E:
        Xe+=[position[edge[0]][0],position[edge[1]][0], None]
        Ye+=[2*M-position[edge[0]][1],2*M-position[edge[1]][1], None]

    labels = v_label


    fig = go.Figure()
    fig.add_trace(go.Scatter(x=Xe,
                    y=Ye,
                    mode='lines',
                    line=dict(color='rgb(210,210,210)', width=5),
                    hoverinfo='none'
                    ))
    fig.add_trace(go.Scatter(x=Xn,
                    y=Yn,
                    mode='markers',
                    name='bla',
                    marker=dict(symbol='square',
                                    size=size,
                                    color=color,
                                    line=dict(color='black', width=1.5)
                                    ),
                    text=labels,
                    hoverinfo='text',
                    ))



    axis = dict(showline=False, # hide axis line, grid, ticklabels and  title
                zeroline=False,
                showgrid=False,
                showticklabels=False,
                )

    fig.update_layout(title= 'Behaviortree',
                annotations=make_annotations(position, v_label, M),
                font_size=15,
                showlegend=False,
                xaxis=axis,
                yaxis=axis,
                margin=dict(l=40, r=40, b=40, t=40),
                hovermode='closest',
                plot_bgcolor='rgb(248,248,248)'
                )
    return fig

