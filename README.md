# RB5_Robotics_Tutorials Github Pages

We created a set of Robotics tutorials for the RB5 Robotics developent platform from Qualcomm. You can visit these tutorials on https://autonomousvehiclelaboratory.github.io/RB5_Robotics_Tutorials/. This repository is for hosting the source code and posts.

## Site maintain

The source code and posts are in branch [hexo-source](https://github.com/AutonomousVehicleLaboratory/RB5_Robotics_Tutorials/tree/hexo-source).
The deployed site is in branch gh-pages.

### How to add posts

Add a new markdown file in the branch hexo-source, folder site/source/_posts/

The Subfolder refers to which category this post belongs to.

### Test and deploy the site

Clone the repository to your local environment and checkout the hexo-source branch.

To run on a local server, run

```
hexo clean && hexo g && hexo s
```

To deploy the site to Github Pages, run
```
hexo clean && hexo g && hexo d
```

This will publish the site to gh-pages branch, without the need to mannually commit and push the code. Wait for a few minutes, then the updates will be available on the website.