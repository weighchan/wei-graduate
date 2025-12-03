## markdown 基础入门

### 1、标题

几个 # 表示几级标题语法，# 与标题之间用空格隔开。  

如：# title —— 一级标题

![image-20250930093717596](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930093717596.png)

### 2、段落

不要用空格（spaces）或制表符（ tabs）缩进段落。

![image-20250930093736241](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930093736241.png)

### 3、换行

在一行的末尾添加两个或多个空格，然后按回车键,即可创建一个换行(`<br>`)。

### 4、强调

**粗体**（Bold）

要加粗文本，请在单词或短语的前后各添加两个星号（asterisks）或下划线（underscores）。

![image-20250930093954280](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930093954280.png)

**斜体（Italic）**

要用斜体显示文本，请在单词或短语前后添加一个星号（asterisk）或下划线（underscore）。要斜体突出单词的中间部分，请在字母前后各添加一个星号，中间不要带空格。

![image-20250930094507575](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930094507575.png)

**粗体（Bold）和斜体（Italic）**

要同时用粗体和斜体突出显示文本，请在单词或短语的前后各添加三个星号或下划线。

![image-20250930094636415](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930094636415.png)

### 5、引用

要创建块引用，请在段落前添加一个 `>` 符号。如下：

>```text
>> Dorothy followed her through many of the beautiful rooms in her castle.
>```

**多个段落的块引用**

块引用可以包含多个段落。为段落之间的空白行添加一个 `>` 符号。如下：

```text
> Dorothy followed her through many of the beautiful rooms in her castle.
>
> The Witch bade her clean the pots and kettles and sweep the floor and keep the fire fed with wood.
```

**嵌套块引用**

块引用可以嵌套。在要嵌套的段落前添加一个 `>>` 符号。如下：  

```text
> Dorothy followed her through many of the beautiful rooms in her castle.
>
>> The Witch bade her clean the pots and kettles and sweep the floor and keep the fire fed with wood.
```

> 11
>
> > 22
>
> 33

**带有其他元素的块引用**

块引用可以包含其他 Markdown 格式的元素。并非所有元素都可以使用，你需要进行实验以查看哪些元素有效。

```text
> #### The quarterly results look great!
>
> - Revenue was off the chart.
> - Profits were higher than ever.
>
>  *Everything* is going according to **plan**.
```

> #### The quarterly results look great!
>
> - Revenue was off the chart.
>
> - Profits were higher than ever.  
> - *Everything* going according to **plan**.

### 6、列表

**有序列表**

要创建有序列表，请在每个列表项前添加数字并紧跟一个英文句点。  

![image-20250930100026026](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930100026026.png)

1. first item
2. second item

**无序列表**

要创建无序列表，请在每个列表项前面添加破折号 (-)、星号 (*) 或加号 (+) 。缩进一个或多个列表项可创建嵌套列表。![image-20250930100257205](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930100257205.png)

**在列表中嵌套其他元素**

要在保留列表连续性的同时在列表中添加另一种元素，请将该元素缩进四个空格或一个制表符，如下例所示：

```text
*   This is the first list item.
*   Here's the second list item.

    I need to add another paragraph below the second list item.

*   And here's the third list item.
```

* 1

  second item.

* 3

**引用块**

```text
*   This is the first list item.
*   Here's the second list item.

    > A blockquote would look great below the second list item.

-   And here's the third list item.
```

* This is the first list item.

  > A blockquote would look great below the second list item.

- And here's the third list item.

### 7、代码语法

要将单词或短语表示为代码，请将其包裹在反引号 (` 代码`) 中。反引号位置位于英文状态下，和波浪线同一个键位。

| Markdown语法                          | HTML                                             | 预览效果                            |
| ------------------------------------- | ------------------------------------------------ | :---------------------------------- |
| `At the command prompt, type `nano`.` | `At the command prompt, type <code>nano</code>.` | At the command prompt, type `nano`. |

**代码块**

要创建代码块，创建一个引用块，将 text 调整为需要的代码语言或者直接创建一个代码块 ` ctrl+shift+k` 后直接选择语言。

> 公式块 ` ctrl+shift+m`

```python
import numpy
import math

j = 0
for i in range(1,8,1):
    i += 1
    print("i")
while(j <= 100):
    j += 1
    
print("hello world!{}".format(j))
```

```c
#include <stdio.h>

int main()
{
    printf("hello world!");
    return 0;
}
```

### 8、分割线

要创建分隔线，请在单独一行上使用三个或多个星号 (`***`)、破折号 (`---`) 或下划线 (`___`) ，并且不能包含其他内容。

![image-20250930101503040](C:\Users\HP\AppData\Roaming\Typora\typora-user-images\image-20250930101503040.png)

### 9、链接

**链接**

链接文本放在中括号内，链接地址放在后面的括号中，链接title可选。

超链接Markdown语法代码：`[超链接显示名](超链接地址 "超链接title")`

对应的HTML代码：`<a href="超链接地址" title="超链接title">超链接显示名</a>`

这是一个链接 [Markdown教程](https://markdown.com.cn)。

```text
这是一个链接 [Markdown语法](https://markdown.com.cn "最好的markdown教程")。
```

这是一个链接 [Markdown教程](https://markdown.com.cn "仅供学习使用")。

**网址和Email地址**

使用尖括号可以很方便地把URL或者email地址变成可点击的链接。

```text
<https://markdown.com.cn>
<fake@example.com>
```

<https://markdown.com.cn>

<weic1018@163.com>

**带格式化的链接**

强调链接, 在链接语法前后增加星号。 要将链接表示为代码，请在方括号中添加反引号。

```text
I love supporting the **[EFF](https://eff.org)**.
This is the *[Markdown Guide](https://www.markdownguide.org)*.
See the section on [`code`](#code).
```

**Tips:**

不同的 Markdown 应用程序处理URL中间的空格方式不一样。为了兼容性，请尽量使用%20代替空格。

| ✅ Do this                                           | ❌ Don't do this                                 |
| --------------------------------------------------- | ----------------------------------------------- |
| `[link](https://www.example.com/my%20great%20page)` | `[link](https://www.example.com/my great page)` |

### 10、图片

**添加图片**

要添加图像，请使用感叹号 (`!`), 然后在方括号增加替代文本，图片链接放在圆括号里，括号里的链接后可以增加一个可选的图片标题文本。

插入图片Markdown语法代码：`![图片alt](图片链接 "图片title")`。

对应的HTML代码：`<img src="图片链接" alt="图片alt" title="图片title">`

![示例图片](C:\Users\HP\Pictures\Saved Pictures\雷电将军2K.jpg "角色-雷电将军")

**链接图片**

```text
[![沙漠中的岩石图片](/assets/img/shiprock.jpg "Shiprock")](https://markdown.com.cn)
```

试着点击图片试一试（链接点击方式：Ctrl + 鼠标左键）。

[![示例图片](C:\Users\HP\Pictures\Saved Pictures\1713523925991.jpg "角色-仆人")](https://gd-hbimg.huaban.com/aad18e82ccf3dc2a5d578b5170dad0ff48f0818a14d56-luiVZN_fw658webp)

### 11、转义字符

要显示原本用于格式化 Markdown 文档的字符，请在字符前面添加反斜杠字符 \ 。

```text
\* Without the backslash, this would be a bullet in an unordered list.
```

\* Without the backslash, this would be a bullet in an unordered list.

**可做转义的字符**

以下列出的字符都可以通过使用反斜杠字符从而达到转义目的。

| Character | Name                                           |
| --------- | ---------------------------------------------- |
| \         | backslash                                      |
| `         | backtick (see also escaping backticks in code) |
| *         | asterisk                                       |
| _         | underscore                                     |
| { }       | curly braces                                   |
| [ ]       | brackets                                       |
| ( )       | parentheses                                    |
| #         | pound sign                                     |
| +         | plus sign                                      |
| -         | minus sign (hyphen)                            |
| .         | dot                                            |
| !         | exclamation mark                               |
| \|        | pipe (see also escaping pipe in tables)        |

### 12、扩展语法

一些个人和组织开始通过添加其他元素（例如表，代码块，语法突出显示，URL自动链接和脚注）来[扩展基本语法](https://markdown.com.cn/extended-syntax/)。可以通过使用基于基本Markdown语法的轻量级标记语言，或通过向兼容的Markdown处理器添加扩展来启用这些元素。
