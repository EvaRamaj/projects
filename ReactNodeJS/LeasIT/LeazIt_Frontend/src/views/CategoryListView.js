"use strict";

import React from 'react';

import { CategoryList } from '../components/CategoryList';

import CategoryService from '../services/CategoryService';


export class CategoryListView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            data: []
        };
    }

    componentWillMount(){
        this.setState({
            loading: true
        });

        CategoryService.getCategories().then((data) => {
            this.setState({
                data: [...data],
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    deleteCategory(id) {
        this.setState({
            data: [...this.state.data],
            loading: true
        });
        CategoryService.deleteCategory(id).then((message) => {

            let categoryIndex = this.state.data.map(category => category['_id']).indexOf(id);
            let categories = this.state.data;
            categories.splice(categoryIndex, 1);
            this.setState({
               data: [...categories],
               loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }


        // --TODO for harilakis with love https://stackoverflow.com/questions/33242378/rendering-react-components-with-promises-inside-the-render-method
        return (
            <CategoryList data={this.state.data} onDelete={(id) => this.deleteCategory(id)}/>
        );
    }
}
