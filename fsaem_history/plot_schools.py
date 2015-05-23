import matplotlib.pyplot as plt
import numpy as np

from pandas import read_csv

# These are the "Tableau 20" colors as RGB.
tableau20 = [(31, 119, 180), (174, 199, 232), (255, 127, 14), (255, 187, 120),
             (44, 160, 44), (152, 223, 138), (214, 39, 40), (255, 152, 150),
             (148, 103, 189), (197, 176, 213), (140, 86, 75), (196, 156, 148),
             (227, 119, 194), (247, 182, 210), (127, 127, 127), (199, 199, 199),
             (188, 189, 34), (219, 219, 141), (23, 190, 207), (158, 218, 229)]

# Scale the RGB values to the [0, 1] range, which is the format matplotlib accepts.
for i in range(len(tableau20)):
    r, g, b = tableau20[i]
    tableau20[i] = (r / 255., g / 255., b / 255.)

# Read in the competition results
competition_results_data = read_csv('FSAE Selected Results.csv')

# Find schools in the data set
schools = competition_results_data.School.unique()
years = competition_results_data.Year.unique()
years = np.sort(years[~np.isnan(years)])

# Create the figure
plt.figure(figsize=(14, 9))
plt.title('Historical Performance of Teams at Formula SAE Michigan')
plt.xlabel('Year')
plt.ylabel('Total Score')
plt.ylim([0, 1000])
plt.subplots_adjust(right=0.72)

# Remove plot frame lines
ax = plt.subplot(111)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()
ax.set_xticks(years)

# In case we have a string, prepare a dictionary to convert points to numeric
mapping = {'withdrawn': 0, 'forfeit': 0}

# Plot data for each school
for idx, team in enumerate(schools):
    team_results = competition_results_data[competition_results_data.School == team].sort_index(by='Year')
    try:
        team_points = team_results.Points.replace(mapping).convert_objects(convert_numeric=True).values
    except:
        team_points = team_results.Points.convert_objects(convert_numeric=True).values

    rand_color = np.random.rand(3,)
    plt.plot(team_results.Year.convert_objects(convert_numeric=True).values, team_points, lw=1.3, color=rand_color)


# Display data
#plt.legend(schools, loc='center left', bbox_to_anchor=(1.03, 0.5), fontsize=11, frameon=False)
plt.savefig('teamresults.png')
plt.show()
