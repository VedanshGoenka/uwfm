import matplotlib.pyplot as plt
import numpy as np

from pandas import read_csv


# Read in the competition results
competition_results_data = read_csv('FSAE Selected Results.csv')

''' Generate Averages '''
# Calculate the mean and standard deviation of each year
competition_years = competition_results_data.Year.unique()
competition_years = np.sort(competition_years[~np.isnan(competition_years)])

competition_statistics = np.empty((competition_years.size, 5))

for idx, year in enumerate(competition_years):
    competition_statistics[idx, 0] = year
    competition_statistics[idx, 1] = competition_results_data[competition_results_data.Year == year].Points.convert_objects(convert_numeric=True).mean()
    competition_statistics[idx, 2] = competition_results_data[competition_results_data.Year == year].Points.convert_objects(convert_numeric=True).std()
    competition_statistics[idx, 3] = competition_results_data[competition_results_data.Year == year].Points.convert_objects(convert_numeric=True).max()
    competition_statistics[idx, 4] = competition_results_data[competition_results_data.Year == year].Points.convert_objects(convert_numeric=True).min()

print(competition_statistics)
# Create the figure
plt.figure(figsize=(12, 9))
plt.title('Historical Total Scores at Formula SAE Michigan')
plt.xlabel('Year')
plt.ylabel('Total Score')

# Remove plot frame lines
ax = plt.subplot(111)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()
ax.set_xticks(competition_years)

# Plot overall data
plt.fill_between(competition_statistics[:, 0], competition_statistics[:, 4], competition_statistics[:, 3], color='#758AA8')
plt.fill_between(competition_statistics[:, 0], competition_statistics[:, 1] - competition_statistics[:, 2],
                 competition_statistics[:, 1] + competition_statistics[:, 2], color='#4C668C')
plt.plot(competition_statistics[:, 0], competition_statistics[:, 1], color='#051A37', lw=2.5)

# Display data
plt.legend(['Mean'])

''' Generate Histograms '''
plt.figure(figsize=(12, 9))
plt.title('Distribution of Total Scores at Formula SAE Michigan since 2001')
plt.xlabel('Total Score')
plt.ylabel('Frequency')

ax = plt.subplot(111)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()

# In case we have a string, prepare a dictionary to convert points to numeric
mapping = {'withdrawn': 0, 'forfeit': 0}

try:
    all_points = competition_results_data.Points.replace(mapping).convert_objects(convert_numeric=True)
except:
    all_points = competition_results_data.Points.convert_objects(convert_numeric=True)

all_points = all_points[~np.isnan(all_points)]
plt.hist(all_points.values, bins = 50);
plt.xlim([0, 1000])

# Plot a single year
plt.figure(figsize=(12, 9))
plt.title('Distribution of Total Scores at Formula SAE Michigan 2014')
plt.xlabel('Total Score')
plt.ylabel('Frequency')

ax = plt.subplot(111)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()

year = 2014

# In case we have a string, prepare a dictionary to convert points to numeric
mapping = {'withdrawn': 0, 'forfeit': 0}
bins = np.linspace(0, 1000, 20)

try:
    year_points = competition_results_data[competition_results_data.Year == year].Points.replace(mapping).convert_objects(convert_numeric=True)
except:
    year_points = competition_results_data[competition_results_data.Year == year].Points.convert_objects(convert_numeric=True)

plt.hist(year_points.values, bins=bins, color='#990000');
plt.xlim([0, 1000])
plt.show()
