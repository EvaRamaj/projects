"use strict";

import React from 'react';

import CategoryForm from './../components/CategoryForm';

import CategoryService from '../services/CategoryService';


export class CategoryFormView extends React.Component {

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


        // if(this.props.history.location.pathname === '/add_category') {
        //     this.setState({
        //         loading: false,
        //         item: undefined,
        //         error: undefined
        //     });
        // }
        // else if(this.props.location.state !== undefined && this.props.location.state.categories !== undefined) {
        //     this.setState({
        //         loading: false,
        //         categories: this.props.location.state.categories,
        //         error: undefined
        //     });
        // }
        // else {
        //     this.setState({
        //         loading: true,
        //         error: undefined
        //     });
        //
        //     let id = this.props.match.params.id;
        //
        //     CategoryService.getCategory(id).then((data) => {
        //         this.setState({
        //             category: data,
        //             loading: false,
        //             error: undefined
        //         });
        //     }).catch((e) => {
        //         console.error(e);
        //     });
        // }
    }

    updateCategory(category) {
        console.log(this.state.categories);
        if(this.state.categories === undefined) {
            CategoryService.createCategory(category).then((data) => {
                this.props.history.push('/add_category');
                window.location.reload();
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
            });
        } else {
            CategoryService.updateCategory(category).then((data) => {
                this.props.history.goBack();
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating movie'}));
            });
        }
    }

    deleteCategory(id) {
        console.log('delete:', id);
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
        return (<CategoryForm category={this.state.item} data={this.state.data} onDelete={(id) => this.deleteCategory(id)} onSubmit={(category) => this.updateCategory(category)} error={this.state.error} />);
    }
}
