Zhuocheng Han｜个人主页

公开地址：https://ganboyu1999.github.io/

文件说明
index.html：中文主页，以工作经历和个人贡献为主。
frs.html：机器人二次开发框架与通信中间件的项目梳理。
fdi.html：数据观测与异常取证的项目梳理。
fviz.html：可视化研发平台的项目梳理。
style.css：窄单栏、衬线字体、响应式布局、系统深浅色与打印样式。
favicon.svg：ZH 图标。
.nojekyll：GitHub Pages 直接托管静态文件。

作品素材补充
主页 #works 下保留 FRS、FDI、FVIZ 三个 .work-item。
将对应 .work-placeholder 换成已确认可公开的截图或视频链接；项目详情页各自保留 .detail-placeholder。
图片和视频放入 assets/，使用相对路径，补充说明、图片替代文本或视频标题。
录屏建议展示问题场景、关键操作和结果；图片建议注明个人负责部分。
未加入演示素材前，页面只显示“素材待补充”，不提供无效播放按钮。

维护与发布
纯 HTML/CSS，无构建步骤、无 JavaScript 和第三方运行依赖。
本地预览：python3 -m http.server 8773
GitHub Pages：main 分支，根目录（/）。
页面采用所有者指定的英文显示名，不包含中文姓名、照片、手机号或原简历下载。
排版参考 https://ddod-jim.github.io/，个人职责及成果以所有者提供的终版简历为准。

本版信息层次
首屏：职业定位、当前团队、邮件入口；代表成果摘要可直达相应项目。
代表项目：问题与价值、个人职责、成果及详细方案入口。
工作经历：当前角色完整展开；Momenta 先呈现摘要，完整贡献通过原生 details 展开。
作品展示：参考 Cillian 的两列媒体分区，保留三组待补充位置。
项目详情：本人职责与成果前置，再展开问题、贡献、设计和技术边界。
robots.txt / sitemap.xml：公开站点的索引入口；canonical 与分享元信息对应各自真实页面。
页面没有第三方跟踪、外部字体或 JavaScript 运行依赖。
