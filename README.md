# RB5_Robotics_Tutorials Github Pages

We created a set of Robotics tutorials for the RB5 Robotics developent platform from Qualcomm. You can visit these tutorials on https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/. This repository is for hosting the source code and posts.

## Site maintain

The source code and posts are in branch [hexo-source](https://github.com/AutonomousVehicleLaboratory/RB5_Robotics_Tutorials/tree/hexo-source).
The deployed site is in branch [gh-pages](https://github.com/AutonomousVehicleLaboratory/RB5_Robotics_Tutorials/tree/gh-pages).

### How to add posts

Posts are organized into categories. These categories are subfolders in the [hexo-source](https://github.com/AutonomousVehicleLaboratory/RB5_Robotics_Tutorials/tree/hexo-source) branch in site/source/_posts/. new posts created in the folder will have the same category as its folder name. 

To create a new post, add a new markdown file in the corresponding category folder. A post should have the title, date and category attributes and optionally having tags. Posts in each category are ordered by (1) (2) (3) indicating the reading order. 

More features can be found in the [theme-documentation](https://github.com/LouisBarranqueiro/hexo-theme-tranquilpeak/blob/master/DOCUMENTATION.md). Adding an image to the pose can be done by using the post-asset-folder, which the a folder with the same name as the post in the same directory where the post is. Put your iamge in it and use the following code in your post.

```
{% image image_name.png [title text] %}
```

Then, you can test your site locally or deploy it to github pages by the following steps. 

### Test and deploy the site

Clone the repository to your local environment and checkout the [hexo-source](https://github.com/AutonomousVehicleLaboratory/RB5_Robotics_Tutorials/tree/hexo-source) branch. Enter the site folder.

Then you need to setup the npm pacakges for the theme that we are using. For this, enter the themes/tranquilpeak folder and run

```
npm install && npm run prod
```

Then return to the site folder.

To run on a local server, run

```
hexo clean && hexo g && hexo s
```

To deploy the site to Github Pages, run
```
hexo clean && hexo g && hexo d
```

This will publish the site to [gh-pages](https://github.com/AutonomousVehicleLaboratory/RB5_Robotics_Tutorials/tree/gh-pages) branch, without the need to mannually commit and push the code. Wait for a few minutes, then the updates will be available on the website.

Notice that even though the deployment branch is updated after the previous step, you still need to commit and push your updates to the 'hexo-source' branch.

## Environment Setup

This website uses Hexo with Github Pages. To enable generating website with hexo, you will need npm and nodejs.

### Robotics people setup

If you have ROS installed or need ROS in the future, there will be a dependency conflict when you are trying to install npm. Therefore a workaround is provided in this [StackOverFlow Answer](https://stackoverflow.com/questions/65519982/can-not-install-npm-along-side-with-the-ros-melodic
). Basically, you need to setup a python virtual environment, in which you will install nodeenv, and use nodeenv to create another virtual environment, that allows you to have npm and nodejs installed without the need to remove all ROS related packages. Steps to create nodeenv virtual environment is in this [link](https://pypi.org/project/nodeenv/#basic). Then you can install npm with apt within this environment.

### Standard Setup

Follow the guide from [Hexo](https://hexo.io/).

## Theme

We are using an Hexo theme [Tranquilpeak](https://github.com/LouisBarranqueiro/hexo-theme-tranquilpeak). This theme is downloaded into the site/themes/tranquilpeak folder.

Many of the settings can be found in the following files:

1. site/_config.yml (site settings succh as title)
2. site/themes/tranquilpeak/_config.yml (theme settings like the sidebar configuration)
3. site/themes/tranquilpeak/languages/en.yml (about page intro)
