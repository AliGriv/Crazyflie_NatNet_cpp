import pandas as pd
import plotly.express as px

### Global Variables, please define them first
numCopters = 1
fileName = 'data.csv'
delimiter_of_file =';'




def create_variables_list(name, id, index_list = []):
    temp_list = []
    idd = str(id)
    name += '['+idd+']'
    if (len(index_list) > 0):
        for c in index_list:
            title = name + c
            temp_list.append(title)
    else:
        temp_list.append(name)

    return temp_list


if __name__ == '__main__':
    plots_name = []  # list of lists, each sub list will be used for a single plot
    time_axis = []
    # Let's generate some sort of grouped list of variable names
    for i in range(numCopters):
        plots_name.append(create_variables_list("DesiredPosition", i, ['x', 'y', 'z']))
        plots_name.append(create_variables_list("Position", i, ['x', 'y', 'z']))
        plots_name.append(create_variables_list("PositionError", i, ['x', 'y', 'z']))
        plots_name.append(create_variables_list("Velocity", i, ['x', 'y', 'z']))
        plots_name.append(create_variables_list("Orientation", i, ['x', 'y', 'z', 'w']))
        plots_name.append(create_variables_list("Command", i, ["roll","pitch","throttle","yawrate"]))
        plots_name.append(create_variables_list("HighLevelCommand", i, ['x', 'y', 'z']))
        plots_name.append(create_variables_list("TrackingFlag", i, []))
        plots_name.append(create_variables_list("Yaw", i, []))
        time_axis.append(create_variables_list("TimeCounter", i, []))
    print(plots_name)
    print(time_axis)

    print("Reading the CSV file")
    df = pd.read_csv('data.csv', delimiter=delimiter_of_file)

    for plt_names in plots_name:
        index = plt_names[0].find('[')
        title = plt_names[0][0:index]
        id = int(plt_names[0][index+1])

        fig = px.line(df, x=time_axis[id][0], y=plt_names, title=title)
        fig.show()







